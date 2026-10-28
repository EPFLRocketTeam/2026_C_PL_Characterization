/* Optional C11 high-throughput decoder for the V2.5.1 on-card format.
 * Called by parser.py after its Python timing-fit pass. Python owns diagnostic
 * decoding and run summary; this executable writes the four high-rate CSVs and
 * interleaved WAV, and records sample-index/tag anomalies.
 * No external libraries. Build: gcc -O3 -std=c11 -D_FILE_OFFSET_BITS=64 -o fast_decode fast_decode.c -lm
 */
#define _FILE_OFFSET_BITS 64
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <errno.h>

#ifdef _WIN32
#define SEEK64 _fseeki64
#define TELL64 _ftelli64
#else
#define SEEK64 fseeko
#define TELL64 ftello
#endif

#define MAX_SENSOR 256
#define MAX_PAYLOAD 4096
#define MAX_EPOCHS 8192
#define WRITER_BUF (1<<20)
#define MOD32 4294967296ULL
#define ERR(...) do {fprintf(stderr,"fast_decode: " __VA_ARGS__);fputc('\n',stderr);goto fail;}while(0)

static uint16_t u16(const unsigned char *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static int16_t i16(const unsigned char *p){return (int16_t)u16(p);}
static uint32_t u32(const unsigned char *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void put16(unsigned char *p,uint16_t x){p[0]=x&255;p[1]=(x>>8)&255;}
static void put32(unsigned char *p,uint32_t x){for(int i=0;i<4;i++)p[i]=(x>>(i*8))&255;}
static int copy_path(char *dst,size_t n,const char *dir,const char *name){
 int z=snprintf(dst,n,"%s/%s",dir,name);return z>=0 && (size_t)z<n;
}
static char file_buffers[8][WRITER_BUF]; static int used_buffers=0;
static FILE *new_file(const char *dir,const char *name){
 char path[4096]; if(!copy_path(path,sizeof(path),dir,name))return NULL;
 FILE *fp=fopen(path,"wb");
 if(fp && used_buffers<8)setvbuf(fp,file_buffers[used_buffers++],_IOFBF,WRITER_BUF);
 return fp;
}

typedef struct {int valid; uint32_t last_raw; uint64_t time;} Clock;
typedef struct {int sid; uint32_t epoch; double rate[2],base[2];} Fit;
typedef struct {
 int sid, fitted, overridden;uint32_t epoch;
 double rate[2],base[2];
 uint32_t next[2], last_index[2]; int tag[2],check_tag[2],seen[2];
 char model[64];
} Epoch;

static Fit fits[MAX_EPOCHS];static size_t nf=0;
static Epoch epochs[MAX_EPOCHS];static size_t ne=0;
static uint64_t gap[2][2], anomaly[2][2],row_count[2][2],index_disc[2][2],tag_disc[2][2];

static void write_wav_header(FILE *wav,uint64_t audio_bytes,uint32_t sample_rate){
 unsigned char h[44]={0};
 memcpy(h,"RIFF",4);put32(h+4,(uint32_t)(audio_bytes+36));memcpy(h+8,"WAVEfmt ",8);
 put32(h+16,16);put16(h+20,1);put16(h+22,2);put32(h+24,sample_rate);
 put32(h+28,sample_rate*4);put16(h+32,4);put16(h+34,16);
 memcpy(h+36,"data",4);put32(h+40,(uint32_t)audio_bytes);
 fwrite(h,1,44,wav);
}
static int read_fits(const char *path){
 if(!path)return 1;
 FILE *fp=fopen(path,"rb");if(!fp)return 0;
 int sid;unsigned ep; double ar,ab,gr,gb;
 while(fscanf(fp,"%d %u %lf %lf %lf %lf",&sid,&ep,&ar,&ab,&gr,&gb)==6){
  if(nf==MAX_EPOCHS){fclose(fp);return 0;}
  fits[nf++]=(Fit){sid,ep,{ar,gr},{ab,gb}};
 }
 int ok=feof(fp);fclose(fp);return ok;
}
static int valid_length(int sid,uint32_t length){
 if(sid==17||sid==18)return length>=28&&length<=28+170*6&&(length-28)%6==0;
 if(sid==19||sid==20)return length>=28&&length<=28+300*7&&(length-28)%7==0;
 if(sid==5||sid==6)return length==12;
 if(sid==7)return length==512;
 if(sid==8||sid==255)return length==0;
 if(sid==48)return length==1004;
 if(sid==49)return length==18;
 if(sid==50)return length==28;
 if(sid==51)return length==24;
 return 0;
}
int main(int argc,char **argv){
 const char *input=NULL,*out=NULL,*fitfile=NULL;int resync=0,summary_only=0;
 double adxl_rate=5120,lsm_rate=6670,rate_main=0,rate_sat=0;
 uint32_t audio_rate=44100;
 for(int i=1;i<argc;i++){
  const char *a=argv[i];
  if(!strcmp(a,"--resync")){resync=1;continue;}
  if(!strcmp(a,"--summary-only")){summary_only=1;continue;}
  if(i+1>=argc){fprintf(stderr,"Missing value for %s\n",a);return 2;}
  const char *v=argv[++i];
  if(!strcmp(a,"--input"))input=v;
  else if(!strcmp(a,"--out"))out=v;
  else if(!strcmp(a,"--fits"))fitfile=v;
  else if(!strcmp(a,"--adxl-rate"))adxl_rate=atof(v);
  else if(!strcmp(a,"--lsm-rate"))lsm_rate=atof(v);
  else if(!strcmp(a,"--lsm-rate-main"))rate_main=atof(v);
  else if(!strcmp(a,"--lsm-rate-sat"))rate_sat=atof(v);
  else if(!strcmp(a,"--audio-rate"))audio_rate=(uint32_t)atoi(v);
  else {fprintf(stderr,"Unknown argument: %s\n",a);return 2;}
 }
 if(!input||!out||adxl_rate<=0||lsm_rate<=0||!audio_rate){fprintf(stderr,"Missing/invalid input or parameters\n");return 2;}
 if(!read_fits(fitfile)){fprintf(stderr,"Invalid timing-model file\n");return 2;}
 FILE *src=NULL,*wav=NULL,*integrity=NULL;
 char *names[]={"adxl371_main.csv","adxl371_sat.csv","lsm_main_accel.csv","lsm_main_gyro.csv","lsm_sat_accel.csv","lsm_sat_gyro.csv"};
 FILE *csvs[6]={0};
 const char *extra="sequence,epoch,sample_index,read_start_us,irq_us,flags_hex,timing_model\n";
 const char *adxl_cols="timestamp_us,raw_x,raw_y,raw_z,g_x,g_y,g_z,";
 const char *acc_cols="timestamp_us,raw_x,raw_y,raw_z,acc_x,acc_y,acc_z,temperature_c,tag_byte,tag_counter,";
 const char *gyr_cols="timestamp_us,raw_x,raw_y,raw_z,gyr_x,gyr_y,gyr_z,temperature_c,tag_byte,tag_counter,";
 src=fopen(input,"rb");if(!src)ERR("Cannot open %s: %s",input,strerror(errno));
 if(!summary_only)for(int i=0;i<6;i++){
  csvs[i]=new_file(out,names[i]);if(!csvs[i])ERR("Cannot create %s",names[i]);
  fprintf(csvs[i],"%s%s",i<2?adxl_cols:(i%2?gyr_cols:acc_cols),extra);
 }
 if(!summary_only){
  wav=new_file(out,"flight_audio.wav");if(!wav)ERR("Cannot create WAV");
  unsigned char blank[44]={0};fwrite(blank,1,44,wav);
 }
 integrity=new_file(out,".native_integrity.tsv");if(!integrity)ERR("Cannot create integrity summary");
 Clock clocks[MAX_SENSOR]={0};uint64_t latest=0,audio_bytes=0;
 unsigned char head[10],payload[MAX_PAYLOAD],stereo[512];
 for(;;){
  int64_t offset=TELL64(src);if(offset<0)ERR("Cannot tell file position");
  size_t nh=fread(head,1,10,src);if(nh==0&&feof(src))break;
  if(nh!=10)ERR("Truncated header at byte %" PRId64,offset);
  int sid=head[1];uint32_t raw_time=u32(head+2),len=u32(head+6);
  if(head[0]!=0xaa||!valid_length(sid,len)){
   if(!resync)ERR("Invalid header at byte %" PRId64,offset);
   if(SEEK64(src,offset+1,SEEK_SET)!=0)ERR("Seek failure during resync");
   continue;
  }
  if(len>MAX_PAYLOAD||fread(payload,1,len,src)!=len)ERR("Truncated payload at byte %" PRId64,offset);
  Clock *clock=&clocks[sid];uint64_t service_time;
  if(clock->valid){
   uint32_t delta=raw_time-clock->last_raw;
   if(delta>=0x80000000U)ERR("Backwards/ambiguous timestamp in sensor %d",sid);
   service_time=clock->time+delta;
  }else if(latest==0)service_time=raw_time;
  else{
   double ep=floor(((double)latest-(double)raw_time+2147483648.0)/4294967296.0);
   service_time=(uint64_t)((int64_t)raw_time+(int64_t)ep*4294967296LL);
  }
  *clock=(Clock){1,raw_time,service_time};if(service_time>latest)latest=service_time;
  if(sid==8||sid==255)break;
  if(sid==7){
   if(summary_only)continue;
   for(int j=0;j<128;j++){
    memcpy(stereo+4*j,payload+2*j,2);memcpy(stereo+4*j+2,payload+256+2*j,2);
   }
   if(audio_bytes+512>UINT32_MAX-36)ERR("WAV exceeds 4GiB RIFF size limit; split recording");
   if(fwrite(stereo,1,512,wav)!=512)ERR("WAV write failed");
   audio_bytes+=512;
   continue;
  }
  if(sid<17||sid>20)continue;
  uint32_t seq=u32(payload),ep=u32(payload+4),irqraw=u32(payload+8),acc=u32(payload+12),gyro=u32(payload+16);
  uint16_t entries=u16(payload+20),count=u16(payload+22),flags=u16(payload+24);int16_t temp=i16(payload+26);
  int adxl=sid<=18;int width=adxl?6:7;
  if(len!=28+(uint32_t)count*width)ERR("Acquisition count/length mismatch at byte %" PRId64,offset);
  if((flags&0xde)&&count)ERR("Acquisition fault packet has data at byte %" PRId64,offset);
  if(!count)continue;
  double irq=0;int valid_irq=(flags&1)!=0;
  if(valid_irq){uint32_t delta=irqraw-(uint32_t)service_time;
   irq=(double)service_time+(double)(int32_t)delta;
  }
  char irq_str[48]="";
  if(valid_irq)snprintf(irq_str,sizeof(irq_str),"%.0f",irq);
  if(adxl){
   if(summary_only)continue;  /* Metadata and packet validation remain in Python. */
   FILE *dest=csvs[sid-17];double dt=1e6/adxl_rate;
   double first=valid_irq?irq-99*dt:(double)service_time-(count-1)*dt;
   const char *model=valid_irq?"watermark_irq_nominal":"read_start_nominal";
   for(int i=0;i<count;i++){
    int16_t x=i16(payload+28+i*6),y=i16(payload+30+i*6),z=i16(payload+32+i*6);
    if(fprintf(dest,"%.17g,%d,%d,%d,%.17g,%.17g,%.17g,%" PRIu32 ",%" PRIu32 ",%d,%" PRIu64 ",%s,0x%x,%s\n",first+i*dt,x,y,z,x*.1,y*.1,z*.1,seq,ep,i,service_time,irq_str,flags,model)<0)ERR("CSV write failure");
   }
   continue;
  }
  Epoch *state=NULL;
  for(size_t j=0;j<ne;j++)if(epochs[j].sid==sid&&epochs[j].epoch==ep){state=&epochs[j];break;}
  if(!state){
   if(ne>=MAX_EPOCHS)ERR("Too many LSM reset epochs");
   state=&epochs[ne++];memset(state,0,sizeof(*state));
   state->sid=sid;state->epoch=ep;state->tag[0]=state->tag[1]=-1;
   state->check_tag[0]=state->check_tag[1]=-1;
   double override=(sid==19?rate_main:rate_sat),rate=override>0?override:lsm_rate;
   double dt=1e6/rate;
   double first=valid_irq?irq-149*dt:(double)service_time-((entries+1)/2>1?((entries+1)/2-1):0)*dt;
   state->overridden=override>0;state->rate[0]=state->rate[1]=rate;
   state->base[0]=first-acc*dt;state->base[1]=first-gyro*dt;
   int fitted=0;
   if(!state->overridden)for(size_t j=0;j<nf;j++){
    if(fits[j].sid==sid&&fits[j].epoch==ep){
     state->rate[0]=fits[j].rate[0];state->rate[1]=fits[j].rate[1];
     state->base[0]=fits[j].base[0];state->base[1]=fits[j].base[1];fitted=1;break;
    }
   }
   state->fitted=fitted;
   snprintf(state->model,sizeof(state->model),"%s",fitted?"epoch_fifo_mcu_affine_fit":
     (state->overridden?(valid_irq?"epoch_user_rate_irq_anchor":"epoch_user_rate_read_start_anchor"):
                        (valid_irq?"epoch_nominal_irq_anchor":"epoch_nominal_read_start_anchor")));
   state->next[0]=acc;state->next[1]=gyro;
  }
  uint32_t inds[]={acc,gyro};int row=(sid==19?0:1);
  for(int k=0;k<2;k++){
   if(inds[k]<state->next[k])ERR("LSM sample index went backwards at byte %" PRId64,offset);
   if(inds[k]>state->next[k]){gap[row][k]+=(uint64_t)(inds[k]-state->next[k]);state->tag[k]=-1;}
  }
  char temp_str[48]="";
  if(!(flags&0x100))snprintf(temp_str,sizeof(temp_str),"%.17g",25+(double)temp/256.0);
  for(int j=0;j<count;j++){
   const unsigned char *p=payload+28+j*7;int tag=p[0];int tag_kind=(tag>>3)&31;
   if(tag_kind!=1&&tag_kind!=2)ERR("Unknown LSM tag at byte %" PRId64,offset);
   int kind=(tag_kind==1?1:0),tag_counter=(tag>>1)&3;
   if(state->tag[kind]>=0&&(tag_counter-state->tag[kind]+4)%4!=1)anomaly[row][kind]++;
   state->tag[kind]=tag_counter;
   uint32_t index=inds[kind]++;
   row_count[row][kind]++;
   if(state->seen[kind] && index != state->last_index[kind]+1)index_disc[row][kind]++;
   state->last_index[kind]=index;state->seen[kind]=1;
   if(state->check_tag[kind]>=0 && (tag_counter-state->check_tag[kind]+4)%4!=1)tag_disc[row][kind]++;
   state->check_tag[kind]=tag_counter;
   int16_t x=i16(p+1),y=i16(p+3),z=i16(p+5);
   double scale=kind?0.07:(0.000976*9.80665);
   double time=state->base[kind]+index*1e6/state->rate[kind];
   if(summary_only)continue;
   FILE *dest=csvs[2+row*2+kind];
   if(fprintf(dest,"%.17g,%d,%d,%d,%.17g,%.17g,%.17g,%s,%d,%d,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu64 ",%s,0x%x,%s\n",time,x,y,z,x*scale,y*scale,z*scale,temp_str,tag,tag_counter,seq,ep,index,service_time,irq_str,flags,state->model)<0)ERR("CSV write failure");
  }
  state->next[0]=inds[0];state->next[1]=inds[1];
 }
 fprintf(integrity,"GAP\t19_accel\t%" PRIu64 "\nGAP\t19_gyro\t%" PRIu64 "\nGAP\t20_accel\t%" PRIu64 "\nGAP\t20_gyro\t%" PRIu64 "\n",gap[0][0],gap[0][1],gap[1][0],gap[1][1]);
 fprintf(integrity,"TAG\t19_accel\t%" PRIu64 "\nTAG\t19_gyro\t%" PRIu64 "\nTAG\t20_accel\t%" PRIu64 "\nTAG\t20_gyro\t%" PRIu64 "\n",anomaly[0][0],anomaly[0][1],anomaly[1][0],anomaly[1][1]);
 const char *labels[2][2]={{"lsm_main_accel.csv","lsm_main_gyro.csv"},
                           {"lsm_sat_accel.csv","lsm_sat_gyro.csv"}};
 for(int loc=0;loc<2;loc++)for(int kind=0;kind<2;kind++){
  fprintf(integrity,"CONT\t%s\t%" PRIu64 "\t%" PRIu64 "\t%" PRIu64 "\n",
          labels[loc][kind],row_count[loc][kind],index_disc[loc][kind],tag_disc[loc][kind]);
 }
 if(!summary_only){
  if(SEEK64(wav,0,SEEK_SET)!=0)ERR("Could not finalize WAV header");
  write_wav_header(wav,audio_bytes,audio_rate);
 }
 if(ferror(src)||ferror(integrity))ERR("I/O failure");
 for(int i=0;i<6;i++)if(csvs[i]){if(fclose(csvs[i]))ERR("CSV close error");csvs[i]=NULL;}
 if(wav){if(fclose(wav))ERR("WAV close error");wav=NULL;}
 if(fclose(integrity))ERR("Integrity report close error");
 integrity=NULL;
 fclose(src);return 0;
fail:
 for(int i=0;i<6;i++)if(csvs[i])fclose(csvs[i]);
 if(wav)fclose(wav);
 if(integrity)fclose(integrity);
 if(src)fclose(src);
 return 1;
}
