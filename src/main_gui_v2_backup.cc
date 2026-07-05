// smart_camera GUI — 4路后台播放+触摸+DRM
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/input.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include "detector.h"
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm_fourcc.h>

typedef struct{unsigned char b,g,r,a;}C4;
static C4 mk(unsigned char r,unsigned char g,unsigned char b){return{b,g,r,0};}
static unsigned char F[128][35];
static void fi(){static bool o=0;if(o)return;o=1;memset(F,0,sizeof(F));
auto D=[](char c,const char*s){for(int i=0;i<35;i++)F[(int)c][i]=s[i]=='1';};
D('0',"0111010001100011000110001100011000101110");D('1',"0010001100001000010000100001000010001110");
D('2',"0111010001000010010000100001000010001111");D('3',"0111010001000010011000001100011000101110");
D('4',"0001000110010101001011110001000010000100");D('5',"1111110000100001111000001000011000101110");
D('6',"0111010001100001111010001100011000101110");D('7',"1111100001000100001000100001000010000100");
D('8',"0111010001100010111010001100011000101110");D('9',"0111010001100011000101111000011000101110");
D('A',"01110100011000111111100011000110001");D('B',"11110100011000111110100011000111110");
D('C',"01110100011000010000100001000101110");D('D',"11100100011000110001100011000111100");
D('E',"11111100001000011110100001000011111");D('F',"11111100001000011110100001000010000");
D('G',"01110100011000010111100011000101110");D('H',"10001100011000111111100011000110001");
D('I',"01110001000010000100001000010001110");D('K',"10001100101010011000101001001010001");
D('L',"10000100001000010000100001000011111");D('M',"10001110111010110101100011000110001");
D('N',"10001110011010110011100011000110001");D('O',"01110100011000110001100011000101110");
D('P',"11110100011000111110100001000010000");D('R',"11110100011000111110100011000110001");
D('S',"01110100011000001110000011000101110");D('T',"11111001000010000100001000010000100");
D('U',"10001100011000110001100011000101110");D('V',"10001100011000101010010100010000100");
D('W',"10001100011000110101101011010101010");D('X',"10001100010101000100010101000110001");
D('Y',"10001100010101000100001000010000100");D('Z',"11111000010001000100010001000011111");
D('%',"00001000000010000100001001000010000");D(':',"00000001000000000000010000000000000");
D('-',"00000000000000000000111110000000000");D(' ',"00000000000000000000000000000000000");
}

struct Drm{int fd=0,fr=0,w,h,pitch,size;uint32_t fb[2],crtc,conn;drmModeModeInfo mode;unsigned char*buf[2];
int init(){fd=open("/dev/dri/card0",O_RDWR);if(fd<0)return-1;
auto*R=drmModeGetResources(fd);
for(int i=0;i<R->count_connectors;i++){auto*c=drmModeGetConnector(fd,R->connectors[i]);if(c&&c->connection==DRM_MODE_CONNECTED&&c->count_modes>0){conn=c->connector_id;mode=c->modes[0];auto*e=drmModeGetEncoder(fd,c->encoder_id);if(e){crtc=e->crtc_id;drmModeFreeEncoder(e);}drmModeFreeConnector(c);break;}drmModeFreeConnector(c);}drmModeFreeResources(R);
w=mode.hdisplay;h=mode.vdisplay;
for(int b=0;b<2;b++){drm_mode_create_dumb c={};c.width=w;c.height=h;c.bpp=32;drmIoctl(fd,DRM_IOCTL_MODE_CREATE_DUMB,&c);if(b==0){pitch=c.pitch;size=c.size;}uint32_t H[4]={(uint32_t)c.handle,0u,0u,0u},P[4]={(uint32_t)c.pitch,0u,0u,0u},O[4]={0u};drmModeAddFB2(fd,w,h,DRM_FORMAT_XRGB8888,H,P,O,&fb[b],0);drm_mode_map_dumb m={};m.handle=c.handle;drmIoctl(fd,DRM_IOCTL_MODE_MAP_DUMB,&m);buf[b]=(unsigned char*)mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,m.offset);}
drmModeSetCrtc(fd,crtc,fb[0],0,0,&conn,1,&mode);return 0;}
void flip(){int bk=fr^1;drmModeSetCrtc(fd,crtc,fb[bk],0,0,&conn,1,&mode);fr=bk;}
void clear(int bk){memset(buf[bk],0,size);}
void L2S(int&sx,int&sy,int lx,int ly){sx=1079-ly;sy=lx;}
void pL(int bk,int lx,int ly,C4 c){int sx,sy;L2S(sx,sy,lx,ly);if(sx>=0&&sx<w&&sy>=0&&sy<h){auto*p=buf[bk]+sy*pitch+sx*4;p[0]=c.b;p[1]=c.g;p[2]=c.r;p[3]=c.a;}}
void fL(int bk,int lx,int ly,int rw,int rh,C4 c){for(int y=0;y<rh;y++)for(int x=0;x<rw;x++)pL(bk,lx+x,ly+y,c);}
void bL(int bk,int lx,int ly,int rw,int rh,C4 c,int t=3){for(int y=0;y<rh;y++)for(int x=0;x<rw;x++)if(x<t||x>=rw-t||y<t||y>=rh-t)pL(bk,lx+x,ly+y,c);}
int tw(const char*s,int sz){int w=0;for(;*s;s++)w+=5*sz+1*sz;return w>0?w-1*sz:0;}
void tL(int bk,int cx,int cy,const char*s,C4 c,int sz=3){for(;*s;s++,cx+=5*sz+1*sz)dC(bk,cx,cy,*s,c,sz);}
void tC(int bk,int lx,int ly,int bw_l,int bh_l,const char*s,C4 c,int sz=3){int w=tw(s,sz);tL(bk,lx+(bw_l-w)/2,ly+(bh_l-7*sz)/2,s,c,sz);}
void dC(int bk,int cx,int cy,char ch,C4 c,int sz=2){auto*f=F[(int)ch];if(ch==32)return;if(ch<32||ch>127||(!f[0]&&!f[1]&&!f[4])){fL(bk,cx,cy,5*sz,7*sz,c);return;}for(int fy=0;fy<7;fy++)for(int fx=0;fx<5;fx++)if(f[fy*5+fx])for(int dy=0;dy<sz;dy++)for(int dx=0;dx<sz;dx++)pL(bk,cx+fx*sz+dx,cy+fy*sz+dy,c);}
void rot_write(int bk,cv::Mat&img,int lw,int lh){int dw=lw,dh=img.rows*lw/img.cols;if(dh>lh){dh=lh;dw=img.cols*lh/img.rows;}int xo=(lw-dw)/2,yo=(lh-dh)/2;for(int y=0;y<dh;y++){int sy=y*img.rows/dh;for(int x=0;x<dw;x++){int sx=x*img.cols/dw;int lx=xo+x,ly=yo+y;int px,py;L2S(px,py,lx,ly);auto*src=img.ptr(sy)+sx*3;if(px>=0&&px<w&&py>=0&&py<h){auto*p=buf[bk]+py*pitch+px*4;p[0]=src[0];p[1]=src[1];p[2]=src[2];p[3]=0;}}}}
};

struct Touch{int fd,x,y,down,clicked;Touch(){fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);}
void poll(){clicked=0;input_event ev;while(read(fd,&ev,sizeof(ev))>0){if(ev.type==EV_ABS){if(ev.code==53)x=ev.value;else if(ev.code==54)y=ev.value;}else if(ev.type==EV_KEY&&ev.code==BTN_TOUCH){if(ev.value==1)down=1;else if(ev.value==0&&down){down=0;clicked=1;}}}}
void landscape(int&lx,int&ly){lx=y;ly=1079-x;}};

static bool is_v(int cls){return cls==1||cls==2||cls==3||cls==5||cls==6||cls==7;}
static void draw_boxes(cv::Mat&f,const std::vector<DetectBox>&bx){for(auto&b:bx){int l=std::max(0,(int)b.left),t=std::max(0,(int)b.top),r=std::min(f.cols-1,(int)b.right),bt=std::min(f.rows-1,(int)b.bottom);unsigned char B,G,R;if(b.class_id==0){B=150;G=0;R=255;}else if(is_v(b.class_id)){B=255;G=80;R=0;}else{B=0;G=255;R=0;}for(int y=t;y<=bt;y++){auto*row=f.ptr(y);bool te=(y-t<4),be=(bt-y<4);for(int x=l;x<=r;x++){bool le=(x-l<4),re=(r-x<4);if((te||be)||(le||re)){row[x*3]=B;row[x*3+1]=G;row[x*3+2]=R;}}}}}


int main(){
    fi();Drm drm;drm.init();Touch touch;Detector det;det.init("yolo11s.rknn",0);
    cv::VideoCapture cap0,cap1,cap2,cap3;
    cap0.open("/root/videos/a.mp4");printf("[c0] %d\n",cap0.isOpened());
    cap1.open("/root/videos/b.mp4");printf("[c1] %d\n",cap1.isOpened());
    cap2.open("/root/videos/c.mp4");printf("[c2] %d\n",cap2.isOpened());
    cap3.open("/root/videos/d.mp4");printf("[c3] %d\n",cap3.isOpened());

    enum{MAIN,VIDEO}state=MAIN;int cam=-1,show_ret=0;
    C4 cols[]={mk(200,40,40),mk(40,180,40),mk(40,100,220),mk(220,180,40)};
    int bw=800,bh=400,g=40,bx=(1920-bw*2-g)/2,by=(1080-bh*2-g)/2;
    struct{int x,y;}btn[4]={{bx,by},{bx+bw+g,by},{bx,by+bh+g},{bx+bw+g,by+bh+g}};
    printf("[GUI] started\n");fflush(stdout);
    cv::Mat frame;

    while(1){touch.poll();int tx,ty;touch.landscape(tx,ty);
        if(state==MAIN){int bk=drm.fr^1;drm.clear(bk);drm.tC(bk,0,0,1920,110,"ATK SMART SURVEILLANCE",mk(255,255,255),5);
            for(int i=0;i<4;i++){drm.fL(bk,btn[i].x,btn[i].y,bw,bh,cols[i]);drm.bL(bk,btn[i].x,btn[i].y,bw,bh,mk(255,255,255),4);char big[2]={(char)('A'+i),0};drm.tC(bk,btn[i].x,btn[i].y+60,bw,200,big,mk(255,255,255),16);drm.tC(bk,btn[i].x,btn[i].y+280,bw,60,"ZONE",mk(220,220,220),5);}
            drm.flip();
            if(touch.clicked){for(int i=0;i<4;i++)if(tx>=btn[i].x&&tx<btn[i].x+bw&&ty>=btn[i].y&&ty<btn[i].y+bh){cam=i;state=VIDEO;show_ret=0;}}
        }else{
            cv::Mat f;
            if(cam==0)cap0.read(f);else if(cam==1)cap1.read(f);else if(cam==2)cap2.read(f);else cap3.read(f);
            if(f.empty()){usleep(30000);continue;}
            f.copyTo(frame);
            std::vector<DetectBox>bx;det.detect(frame,bx);draw_boxes(frame,bx);
            int bk=drm.fr^1;drm.clear(bk);drm.rot_write(bk,frame,1920,1080);
            drm.fL(bk,20,20,80,50,mk(0,0,0));drm.bL(bk,20,20,80,50,mk(150,150,150));drm.tL(bk,28,28,"SET",mk(200,200,200),2);
            if(show_ret){drm.fL(bk,20,80,180,50,mk(50,0,0));drm.bL(bk,20,80,180,50,mk(200,100,100));drm.tL(bk,28,90,"BACK",mk(255,200,200),2);}
            drm.flip();
            if(touch.clicked){if(tx>=20&&tx<=100&&ty>=20&&ty<=70)show_ret=!show_ret;if(show_ret&&tx>=20&&tx<=200&&ty>=80&&ty<=130){state=MAIN;show_ret=0;}}
        }usleep(30000);}
    det.release();return 0;
}
