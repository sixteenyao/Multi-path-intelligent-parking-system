// smart_camera GUI v9 — 4路后台播放+触摸+DRM+FreeType系统字体+中文UI
// v1: 像素字体, 单通道
// v2: 像素字体, 4通道选择, show_ret切换返回
// v3: FreeType系统字体, 4通道选择, 直接返回按钮, 时间显示, 缓冲帧即时切换
// v4: step=2跳像素渲染, NPU检测跳帧, usleep 3ms, 竖屏视频兼容
// v5: 消除闪烁 + 帧定时调度(原视频FPS不变)
// v6: 按钮中心点定位文字, 右下角FPS+NPU耗时叠加
// v7: 4路不规则多边形蒙版(A/B/C/D)
// v8: VIRAT_Datasets真实监控数据(65/44/18/24分钟)
// v9: rot_write修复居中偏移, A通道独立draw_mask_A(可缩放), B/C/D帧→画布过滤

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <linux/input.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <chrono>
#include "detector.h"
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm_fourcc.h>
#include <ft2build.h>
#include FT_FREETYPE_H

typedef struct{unsigned char b,g,r,a;}C4;
static C4 mk(unsigned char r,unsigned char g,unsigned char b){return{b,g,r,0};}

// ======================== FreeType 系统字体 ========================
static FT_Library ft_lib;
static FT_Face ft_face;
static void ft_init(){
    if(FT_Init_FreeType(&ft_lib)){printf("[FT] init failed\n");return;}
    // 优先用 Noto Sans SC (支持中文), 备选 Vera
    if(FT_New_Face(ft_lib,"/usr/share/fonts/noto-sans-sc/NotoSansSC-Regular.otf",0,&ft_face)){
        if(FT_New_Face(ft_lib,"/usr/share/fonts/ttf-bitstream-vera/Vera.ttf",0,&ft_face)){
            printf("[FT] no font found\n");
        }
    }
    printf("[FT] font loaded\n");fflush(stdout);
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
void pS(int bk,int sx,int sy,C4 c){if(sx>=0&&sx<w&&sy>=0&&sy<h){auto*p=buf[bk]+sy*pitch+sx*4;p[0]=c.b;p[1]=c.g;p[2]=c.r;p[3]=c.a;}}
void fL(int bk,int lx,int ly,int rw,int rh,C4 c){for(int y=0;y<rh;y++)for(int x=0;x<rw;x++)pL(bk,lx+x,ly+y,c);}
void bL(int bk,int lx,int ly,int rw,int rh,C4 c,int t=3){for(int y=0;y<rh;y++)for(int x=0;x<rw;x++)if(x<t||x>=rw-t||y<t||y>=rh-t)pL(bk,lx+x,ly+y,c);}
	void rot_write(int bk,cv::Mat&img,int lw,int lh){
	    if(img.empty()||img.cols<=0||img.rows<=0)return;
	    int dw=lw,dh=img.rows*lw/img.cols;
	    if(dh>lh){dh=lh;dw=img.cols*lh/img.rows;}
	    int xo=(lw-dw)/2,yo=(lh-dh)/2; // 居中偏移, v4漏了导致A通道错位
	    const int S=2;
	    for(int ly=0;ly<dh;ly+=S){
	        int sy0=ly*img.rows/dh;
	        for(int lx=0;lx<dw;lx+=S){
	            int sx0=lx*img.cols/dw;
	            auto*src=img.ptr(sy0)+sx0*3;
	            int px,py;L2S(px,py,xo+lx,yo+ly);
	            if(px<0||px>=w||py<0||py>=h)continue;
	            auto*p=buf[bk]+py*pitch+px*4;
	            p[0]=src[0];p[1]=src[1];p[2]=src[2];p[3]=0;
	        }
	    }
	}

};

// FreeType 文字绘制: 在横屏逻辑坐标(lx,ly)绘制UTF-8字符串，字号px像素
static void ft_text(Drm& d,int bk,int lx,int ly,const char*u8,int px,C4 clr){
    if(!ft_face)return;
    FT_Set_Pixel_Sizes(ft_face,0,px);
    int sx,sy; d.L2S(sx,sy,lx,ly);
    for(const char*p=u8;*p;){
        unsigned w=0;
        if((unsigned char)*p<128){w=*p;p++;}
        else{int len=0;if((*p&0xE0)==0xC0)len=2;else if((*p&0xF0)==0xE0)len=3;else if((*p&0xF8)==0xF0)len=4;
            unsigned char b[4]={};for(int j=0;j<len;j++)b[j]=*p++;
            if(len==2)w=((b[0]&0x1F)<<6)|(b[1]&0x3F);
            else if(len==3)w=((b[0]&0x0F)<<12)|((b[1]&0x3F)<<6)|(b[2]&0x3F);
            else if(len==4)w=((b[0]&0x07)<<18)|((b[1]&0x3F)<<12)|((b[2]&0x3F)<<6)|(b[3]&0x3F);
        }
        FT_Load_Char(ft_face,w,FT_LOAD_RENDER);
        auto*bm=&ft_face->glyph->bitmap;
        int gx=sx+ft_face->glyph->bitmap_left;
        int gy=sy-ft_face->glyph->bitmap_top;
        for(unsigned ry=0;ry<bm->rows;ry++)for(unsigned rx=0;rx<bm->width;rx++){
            unsigned char a=bm->buffer[ry*bm->pitch+rx];
            if(a>128)d.pS(bk,gx-(int)ry,gy+(int)rx,clr);
        }
        sx-=ft_face->glyph->advance.y>>6;
        sy+=ft_face->glyph->advance.x>>6;
    }
}

// 测量UTF-8字符串宽度(像素)
static int ft_width(const char*u8,int px){
    if(!ft_face)return 0;
    FT_Set_Pixel_Sizes(ft_face,0,px);
    int w=0;
    for(const char*p=u8;*p;){
        unsigned cw=0;
        if((unsigned char)*p<128){cw=*p;p++;}
        else{int len=0;if((*p&0xE0)==0xC0)len=2;else if((*p&0xF0)==0xE0)len=3;else if((*p&0xF8)==0xF0)len=4;
            unsigned char b[4]={};for(int j=0;j<len;j++)b[j]=*p++;
            if(len==2)cw=((b[0]&0x1F)<<6)|(b[1]&0x3F);
            else if(len==3)cw=((b[0]&0x0F)<<12)|((b[1]&0x3F)<<6)|(b[2]&0x3F);
            else if(len==4)cw=((b[0]&0x07)<<18)|((b[1]&0x3F)<<12)|((b[2]&0x3F)<<6)|(b[3]&0x3F);
        }
        FT_Load_Char(ft_face,cw,FT_LOAD_RENDER);
        w+=ft_face->glyph->advance.x>>6;
    }
    return w;
}

// 居中绘制文字 (水平+垂直居中)
static void ft_text_center(Drm& d,int bk,int lx,int ly,int bw,int bh,const char*u8,int px,C4 clr){
    int tw=ft_width(u8,px);
    ft_text(d,bk,lx+(bw-tw)/2,ly+bh/2,u8,px,clr);
}

// 右对齐绘制文字
static void ft_text_right(Drm& d,int bk,int rx,int ly,const char*u8,int px,C4 clr){
    int tw=ft_width(u8,px);
    ft_text(d,bk,rx-tw,ly,u8,px,clr);
}

struct Touch{int fd,x,y,down,clicked;Touch():fd(-1),x(0),y(0),down(0),clicked(0){fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);}
void poll(){clicked=0;input_event ev;while(read(fd,&ev,sizeof(ev))>0){if(ev.type==EV_ABS){if(ev.code==53)x=ev.value;else if(ev.code==54)y=ev.value;}else if(ev.type==EV_KEY&&ev.code==BTN_TOUCH){if(ev.value==1)down=1;else if(ev.value==0&&down){down=0;clicked=1;}}}}
void landscape(int&lx,int&ly){lx=y;ly=1079-x;}};

// 射线法: 判断点是否在多边形内
	static bool pt_in_poly(int px,int py,const std::vector<std::pair<int,int>>&poly){
	    bool inside=false;int n=poly.size();
	    for(int i=0,j=n-1;i<n;j=i++){int xi=poly[i].first,yi=poly[i].second,xj=poly[j].first,yj=poly[j].second;
	        if(((yi>py)!=(yj>py))&&(px<(xj-xi)*(py-yi)/(yj-yi)+xi))inside=!inside;}
	    return inside;
	}
	// 在1920x1080画布上画多边形轮廓(步进法, 5px粗线)
	void draw_poly(Drm&d,int bk,const std::vector<std::pair<int,int>>&poly,C4 c){
	    int n=poly.size();if(n<2)return;
	    for(int i=0;i<n;i++){int j=(i+1)%n;
	        int x1=poly[i].first,y1=poly[i].second,x2=poly[j].first,y2=poly[j].second;
	        int steps=std::max(abs(x2-x1),abs(y2-y1));if(steps<1)steps=1;
	        for(int s=0;s<=steps;s++){
	            int x=x1+(x2-x1)*s/steps, y=y1+(y2-y1)*s/steps;
	            d.fL(bk,x-2,y-2,5,5,c);
	        }
	    }
	}
	// A通道专用: 1920x1080画布上画蒙版轮廓(7px粗线, 坐标×1.506)
	static void draw_mask_A(Drm&d,int bk,const std::vector<std::pair<int,int>>&poly){
	    int n=poly.size();if(n<2)return;
	    C4 yl=mk(255,255,0);
	    float scl=1.506f;
	    for(int i=0;i<n;i++){int j=(i+1)%n;
	        int x1=poly[i].first*scl,y1=poly[i].second*scl,x2=poly[j].first*scl,y2=poly[j].second*scl;
	        int steps=std::max(abs(x2-x1),abs(y2-y1));if(steps<1)steps=1;
	        for(int s=0;s<=steps;s++){
	            int x=x1+(x2-x1)*s/steps, y=y1+(y2-y1)*s/steps;
	            d.fL(bk,x-3,y-3,7,7,yl);
	        }
	    }
	}

static bool is_v(int cls){return cls==1||cls==2||cls==3||cls==5||cls==6||cls==7;}
static void draw_boxes(cv::Mat&f,const std::vector<DetectBox>&bx){for(auto&b:bx){int l=std::max(0,(int)b.left),t=std::max(0,(int)b.top),r=std::min(f.cols-1,(int)b.right),bt=std::min(f.rows-1,(int)b.bottom);unsigned char B,G,R;if(b.class_id==0){B=150;G=0;R=255;}else if(is_v(b.class_id)){B=255;G=80;R=0;}else{B=0;G=255;R=0;}for(int y=t;y<=bt;y++){auto*row=f.ptr(y);bool te=(y-t<4),be=(bt-y<4);for(int x=l;x<=r;x++){bool le=(x-l<4),re=(r-x<4);if((te||be)||(le||re)){row[x*3]=B;row[x*3+1]=G;row[x*3+2]=R;}}}}}

int main(){
    // 杀 Weston 释放 DRM
    system("pkill -9 weston 2>/dev/null");usleep(600000);

    // 初始化 FreeType 系统字体
    ft_init();

    Drm drm;if(drm.init()<0){printf("DRM init failed\n");return 1;}
    Touch touch;
    Detector det;
    if(!det.init("/root/yolo11s.rknn",0)){printf("RKNN init failed\n");return 1;}
    printf("[DET] model loaded\n");fflush(stdout);

    cv::VideoCapture cap0,cap1,cap2,cap3;
    cap0.open("/userdata/videos/a.mp4");printf("[c0] %d\n",cap0.isOpened());
    cap1.open("/userdata/videos/b.mp4");printf("[c1] %d\n",cap1.isOpened());
    cap2.open("/userdata/videos/c.mp4");printf("[c2] %d\n",cap2.isOpened());
    cap3.open("/userdata/videos/d.mp4");printf("[c3] %d\n",cap3.isOpened());
    // 帧定时: 按25fps(40ms/帧)调度, 保证与原视频播放速度一致
    int frame_interval_us[4]={40000,40000,40000,40000};

    enum{MAIN,VIDEO}state=MAIN;int cam=-1;
    // 按钮: A-F 左侧竖向排列
    int bw=360,bh=100,gap=16,lx=50,ly0=140;
    struct{int x,y;}btn[6];
    for(int i=0;i<6;i++){btn[i].x=lx;btn[i].y=ly0+i*(bh+gap);}
    C4 btn_color=mk(15,15,30), btn_gray=mk(60,60,60);
    // 检测蒙版: 每路摄像头可定义不规则多边形, 只检测区域内目标
    std::vector<std::pair<int,int>> masks[4];
    masks[0]={{0,343},{726,275},{1278,330},{751,567},{0,434}}; // A通道蒙版
    masks[1]={{0,349},{685,265},{1919,361},{1919,1079},{0,1079}}; // B通道蒙版
    masks[2]={{458,0},{917,0},{1716,424},{1558,1079},{647,1079}}; // C通道蒙版
    masks[3]={{0,493},{1333,448},{1919,879},{1919,1079},{0,1079}}; // D通道蒙版
    cv::Mat frame,fb0,fb1,fb2,fb3; int bg=0;
    {cv::Mat t;cap0.read(t);if(!t.empty())t.copyTo(fb0);
     cap1.read(t);if(!t.empty())t.copyTo(fb1);
     cap2.read(t);if(!t.empty())t.copyTo(fb2);
     cap3.read(t);if(!t.empty())t.copyTo(fb3);}
    printf("[GUI] started, %dx%d\n",drm.w,drm.h);fflush(stdout);

    int empty_cnt=0, det_skip=0, fps_cnt=0, time_skip=0;
    char tb_time[32]={}, tb_date[32]={};
    {time_t now=time(0);struct tm*lt=localtime(&now);
     strftime(tb_time,32,"%H:%M:%S",lt);strftime(tb_date,32,"%Y.%m.%d",lt);}
    std::vector<DetectBox> last_boxes;
    char fps_str[16]={"FPS: --"}, det_str[16]={"NPU: --ms"}, car_str[16]={"CAR:0"}, per_str[16]={"PER:0"};
    int det_ms=0, cnt_car_acc=0, cnt_per_acc=0, avg_cnt=0;
    auto fps_t0=std::chrono::steady_clock::now();
    auto next_frame_time=std::chrono::steady_clock::now(); // 帧定时调度

    while(1){
        touch.poll();int tx,ty;touch.landscape(tx,ty);

        // 后台轮读: 每轮读一路非活跃摄像头, 4路视频始终前进
        {cv::Mat t;int c=bg;bg=(bg+1)%4;
         if(c!=cam||state==MAIN){
            if(c==0)cap0.read(t);else if(c==1)cap1.read(t);else if(c==2)cap2.read(t);else cap3.read(t);
            if(!t.empty()){if(c==0)t.copyTo(fb0);else if(c==1)t.copyTo(fb1);else if(c==2)t.copyTo(fb2);else t.copyTo(fb3);}
         }}

        if(state==MAIN){
            int bk=drm.fr^1;drm.clear(bk);
            // 顶部标题栏背景
            drm.fL(bk,0,0,1920,100,mk(15,15,30));
            // 标题 — 水平居中
            ft_text_center(drm,bk,0,8,1920,60,"ATK 安防监控系统",36,mk(255,255,255));
            // 时间 — 右上角右对齐
            {time_t now=time(0);struct tm*lt=localtime(&now);char tb[32];
             strftime(tb,32,"%H:%M:%S",lt);ft_text_right(drm,bk,1890,15,tb,26,mk(220,220,220));
             strftime(tb,32,"%Y.%m.%d",lt);ft_text_right(drm,bk,1890,50,tb,20,mk(150,150,150));}
            // 左侧按钮 A-F: A-D可点击, E-F灰色装饰
            for(int i=0;i<6;i++){
                C4 clr=(i<4)?btn_color:btn_gray;
                drm.fL(bk,btn[i].x,btn[i].y,bw,bh,clr);
                drm.bL(bk,btn[i].x,btn[i].y,bw,bh,mk(255,255,255),3);
                char big[2]={(char)('A'+i),0};
                C4 tc=(i<4)?mk(255,255,255):mk(120,120,120);
                int cx=btn[i].x+120, cy=btn[i].y+10;
                ft_text(drm,bk,cx-ft_width(big,80)/2,cy+20,big,80,tc);
            }
            drm.flip();
            if(touch.clicked){for(int i=0;i<4;i++)if(tx>=btn[i].x&&tx<btn[i].x+bw&&ty>=btn[i].y&&ty<btn[i].y+bh){
                cam=i;state=VIDEO;empty_cnt=0;
                cnt_car_acc=cnt_per_acc=avg_cnt=0;snprintf(car_str,16,"CAR:0");snprintf(per_str,16,"PER:0");
                next_frame_time=std::chrono::steady_clock::now();
                // 用已缓冲帧立即显示，避免黑屏等待
                if(cam==0)fb0.copyTo(frame);else if(cam==1)fb1.copyTo(frame);
                else if(cam==2)fb2.copyTo(frame);else fb3.copyTo(frame);
                {int bk2=drm.fr^1;drm.clear(bk2);
                 drm.rot_write(bk2,frame,1920,1080);
                 drm.fL(bk2,20,20,90,50,mk(50,0,0));drm.bL(bk2,20,20,90,50,mk(200,100,100));
                 ft_text(drm,bk2,85-ft_width("返回",22)/2,38,"返回",22,mk(255,200,200));
                 drm.flip();}
                break;}}
            usleep(16000);
        }else{
            cv::Mat f; bool ok=false;
            if(cam==0)ok=cap0.read(f);else if(cam==1)ok=cap1.read(f);
            else if(cam==2)ok=cap2.read(f);else ok=cap3.read(f);
            if(!ok||f.empty()){
                empty_cnt++;
                if(empty_cnt>60){
                    printf("[GUI] no video signal, back to MAIN\n");fflush(stdout);
                    state=MAIN; cam=-1; empty_cnt=0;
                }
                usleep(3000);continue;
            }
            empty_cnt=0;
            f.copyTo(frame);
            // NPU检测跳帧: 每3帧更新一次框位置, 但每帧都画(消除闪烁)
            det_skip=(det_skip+1)%3;
            if(det_skip==0){last_boxes.clear();auto t1=std::chrono::steady_clock::now();det.detect(frame,last_boxes);auto t2=std::chrono::steady_clock::now();det_ms=(int)std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();snprintf(det_str,16,"NPU: %dms",det_ms);
                if(!masks[cam].empty()){
                    std::vector<DetectBox> filtered;
                    for(auto&b:last_boxes){
                        int cx=(b.left+b.right)/2,cy=(b.top+b.bottom)/2;
                        if(cam!=0){ // B/C/D: 帧→画布坐标转换
                            int dww=1920,dhh=frame.rows*1920/frame.cols;
                            if(dhh>1080){dhh=1080;dww=frame.cols*1080/frame.rows;}
                            int xoo=(1920-dww)/2,yoo=(1080-dhh)/2;
                            cx=xoo+cx*dww/frame.cols;cy=yoo+cy*dhh/frame.rows;
                        } // A(cam==0): 保持简单对比
                        if(pt_in_poly(cx,cy,masks[cam]))filtered.push_back(b);
                    }
                    last_boxes=filtered;
                }
            }
            // 统计蒙版内车/人数量, 累加10帧取平均
            {int cnt_car=0,cnt_per=0;
            for(auto&b:last_boxes){if(b.class_id==0)cnt_per++;else if(is_v(b.class_id))cnt_car++;}
            cnt_car_acc+=cnt_car;cnt_per_acc+=cnt_per;avg_cnt++;
            if(avg_cnt>=30){
                snprintf(car_str,16,"CAR:%d",(int)(cnt_car_acc/30.0+0.5));
                snprintf(per_str,16,"PER:%d",(cnt_per_acc+29)/30);
                cnt_car_acc=cnt_per_acc=avg_cnt=0;
            }}
            draw_boxes(frame,last_boxes);
            int bk=drm.fr^1;drm.clear(bk);
            // 左下角: CAR/PER/NPU/FPS 左对齐等间距
            const int IX=20,IFS=22,IDY=22,IY0=1000;
            ft_text(drm,bk,IX,IY0,car_str,IFS,mk(0,80,255));
            ft_text(drm,bk,IX,IY0+IDY,per_str,IFS,mk(255,0,150));
            ft_text(drm,bk,IX,IY0+IDY*2,det_str,IFS,mk(255,200,0));
            ft_text(drm,bk,IX,IY0+IDY*3,fps_str,IFS,mk(0,255,0));
            // 时间: 每30帧更新字符串, 但每帧都渲染(不闪)
            time_skip=(time_skip+1)%30;
            if(time_skip==0){
                time_t now=time(0);struct tm*lt=localtime(&now);
                strftime(tb_time,32,"%H:%M:%S",lt);
                strftime(tb_date,32,"%Y.%m.%d",lt);
            }
            ft_text_right(drm,bk,1890,15,tb_time,26,mk(220,220,220));
            ft_text_right(drm,bk,1890,50,tb_date,20,mk(150,150,150));
            drm.rot_write(bk,frame,1920,1080);
            if(cam==0&&!masks[0].empty())draw_mask_A(drm,bk,masks[0]);
            else if(!masks[cam].empty())draw_poly(drm,bk,masks[cam],mk(255,255,0));
            // 返回按钮 — 文字居中
            drm.fL(bk,20,20,90,50,mk(50,0,0));drm.bL(bk,20,20,90,50,mk(200,100,100));
            ft_text(drm,bk,85-ft_width("返回",22)/2,38,"返回",22,mk(255,200,200));
            drm.flip();
            // FPS统计 (每2秒输出)
            fps_cnt++;
            auto fps_now=std::chrono::steady_clock::now();
            int fps_ms=std::chrono::duration_cast<std::chrono::milliseconds>(fps_now-fps_t0).count();
            if(fps_ms>=2000){double fps=fps_cnt*1000.0/fps_ms;snprintf(fps_str,16,"FPS: %.0f",fps);printf("[FPS] %.1f\n",fps);fflush(stdout);fps_cnt=0;fps_t0=fps_now;}
            if(touch.clicked){if(tx>=20&&tx<=110&&ty>=20&&ty<=70){state=MAIN;cam=-1;empty_cnt=0;}}
            // 帧定时: 按原视频FPS调度, 保持播放速度一致
            if(state==VIDEO){
                next_frame_time+=std::chrono::microseconds(frame_interval_us[cam]);
                auto now=std::chrono::steady_clock::now();
                if(next_frame_time>now)std::this_thread::sleep_until(next_frame_time);
                else next_frame_time=now;
            }
        }}
    det.release();return 0;
}
