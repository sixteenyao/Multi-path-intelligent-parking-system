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
static FT_Face ft_face_cn; // 中文字体 Noto Sans SC
static FT_Face ft_face_en; // 英文字体 Bitstream Vera
static void ft_init(){
    if(FT_Init_FreeType(&ft_lib)){printf("[FT] init failed\n");return;}
    bool cn_ok=(FT_New_Face(ft_lib,"/usr/share/fonts/noto-sans-sc/NotoSansSC-Regular.otf",0,&ft_face_cn)==0);
    bool en_ok=(FT_New_Face(ft_lib,"/usr/share/fonts/ttf-bitstream-vera/Vera.ttf",0,&ft_face_en)==0);
    printf("[FT] CN:%s EN:%s\n",cn_ok?"NotoSansSC":"none",en_ok?"Vera":"none");fflush(stdout);
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
    int sx,sy; d.L2S(sx,sy,lx,ly);
    for(const char*p=u8;*p;){
        unsigned w=0;bool is_ascii=false;
        if((unsigned char)*p<128){w=*p;p++;is_ascii=true;}
        else{int len=0;if((*p&0xE0)==0xC0)len=2;else if((*p&0xF0)==0xE0)len=3;else if((*p&0xF8)==0xF0)len=4;
            unsigned char b[4]={};for(int j=0;j<len;j++)b[j]=*p++;
            if(len==2)w=((b[0]&0x1F)<<6)|(b[1]&0x3F);
            else if(len==3)w=((b[0]&0x0F)<<12)|((b[1]&0x3F)<<6)|(b[2]&0x3F);
            else if(len==4)w=((b[0]&0x07)<<18)|((b[1]&0x3F)<<12)|((b[2]&0x3F)<<6)|(b[3]&0x3F);
        }
        FT_Face face=is_ascii&&ft_face_en?ft_face_en:ft_face_cn;
        if(!face)return;
        FT_Set_Pixel_Sizes(face,0,px);
        FT_Load_Char(face,w,FT_LOAD_RENDER);
        auto*bm=&face->glyph->bitmap;
        int gx=sx+face->glyph->bitmap_left;
        int gy=sy-face->glyph->bitmap_top;
        for(unsigned ry=0;ry<bm->rows;ry++)for(unsigned rx=0;rx<bm->width;rx++){
            unsigned char a=bm->buffer[ry*bm->pitch+rx];
            if(a>128)d.pS(bk,gx-(int)ry,gy+(int)rx,clr);
        }
        sx-=face->glyph->advance.y>>6;
        sy+=face->glyph->advance.x>>6;
    }
}

// 测量UTF-8字符串宽度(像素)
static int ft_width(const char*u8,int px){
    int w=0;
    for(const char*p=u8;*p;){
        unsigned cw=0;bool is_ascii=false;
        if((unsigned char)*p<128){cw=*p;p++;is_ascii=true;}
        else{int len=0;if((*p&0xE0)==0xC0)len=2;else if((*p&0xF0)==0xE0)len=3;else if((*p&0xF8)==0xF0)len=4;
            unsigned char b[4]={};for(int j=0;j<len;j++)b[j]=*p++;
            if(len==2)cw=((b[0]&0x1F)<<6)|(b[1]&0x3F);
            else if(len==3)cw=((b[0]&0x0F)<<12)|((b[1]&0x3F)<<6)|(b[2]&0x3F);
            else if(len==4)cw=((b[0]&0x07)<<18)|((b[1]&0x3F)<<12)|((b[2]&0x3F)<<6)|(b[3]&0x3F);
        }
        FT_Face face=is_ascii&&ft_face_en?ft_face_en:ft_face_cn;
        if(!face)return 0;
        FT_Set_Pixel_Sizes(face,0,px);
        FT_Load_Char(face,cw,FT_LOAD_RENDER);
        w+=face->glyph->advance.x>>6;
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
// 判断点是否在一组多边形中的任意一个内
static bool pt_in_any_poly(int px,int py,const std::vector<std::vector<std::pair<int,int>>>&polys){
    for(auto&p:polys)if(pt_in_poly(px,py,p))return true;
    return false;
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
// 绘制一组多边形
static void draw_all_polys(Drm&d,int bk,const std::vector<std::vector<std::pair<int,int>>>&polys,C4 c){
    for(auto&p:polys)draw_poly(d,bk,p,c);
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

// ======================== 车辆进出检测 (简化版: IOU跟踪 + 位置变化事件) ========================
// 逻辑:
//   1. IOU匹配维护每辆车的稳定ID
//   2. 每辆车记录last_pos: 0=蒙版外, 1=蒙版内
//   3. 已存在ID: last_pos从0→1触发"驶入", 从1→0触发"驶出"
//   4. 新ID首次出现: 不触发事件(不知道它从哪里来)
//   5. 检测丢失期间: last_pos保持不变, 重新匹配后比较变化
//   6. 丢失超过MAX_LOST_FRAMES帧则删除track
//   7. 同一ID最多触发1次驶入+1次驶出, 防止重复事件

static constexpr int MAX_LOST_FRAMES = 10;   // track丢失保留帧数
static constexpr float IOU_THRESHOLD = 0.12f;

struct CarTrack{
    int id;
    DetectBox box;
    int last_pos;      // 上一次位置: 0=外, 1=内
    int lost_frames;   // 连续未匹配帧数
    bool enter_done;   // 已触发驶入事件
    bool exit_done;    // 已触发驶出事件
    CarTrack():id(0),last_pos(0),lost_frames(0),enter_done(false),exit_done(false){}
};

struct CarTracker{
    std::vector<CarTrack> tracks;
    int next_id;

    CarTracker():next_id(1){}

    float iou(const DetectBox&a,const DetectBox&b){
        float ax1=a.left,ay1=a.top,ax2=a.right,ay2=a.bottom;
        float bx1=b.left,by1=b.top,bx2=b.right,by2=b.bottom;
        float ix1=std::max(ax1,bx1),iy1=std::max(ay1,by1);
        float ix2=std::min(ax2,bx2),iy2=std::min(ay2,by2);
        float iw=std::max(0.f,ix2-ix1),ih=std::max(0.f,iy2-iy1),ia=iw*ih;
        float ua=(ax2-ax1)*(ay2-ay1)+(bx2-bx1)*(by2-by1)-ia;
        return ua>0?ia/ua:0;
    }

    struct Event{int id;bool is_enter;};

    void update(const std::vector<DetectBox>&dets,const std::vector<std::vector<std::pair<int,int>>>&mask_list,
                const cv::Mat&frame,int cam_id,std::vector<Event>&events){
        events.clear();
        int n=tracks.size(),m=dets.size();

        // ---- 第1步: IOU贪心匹配 ----
        std::vector<bool>used(m,false);
        for(int i=0;i<n;i++){
            float best=IOU_THRESHOLD;
            int bj=-1;
            for(int j=0;j<m;j++){
                if(used[j])continue;
                float ii=iou(tracks[i].box,dets[j]);
                if(ii>best){best=ii;bj=j;}
            }
            if(bj>=0){
                used[bj]=true;
                tracks[i].box=dets[bj];
                tracks[i].lost_frames=0;
            }else{
                tracks[i].lost_frames++;
            }
        }

        // ---- 第2步: 坐标转换参数 ----
        int dww=1920,dhh=frame.rows*1920/frame.cols;
        if(dhh>1080){dhh=1080;dww=frame.cols*1080/frame.rows;}
        int xoo=(1920-dww)/2,yoo=(1080-dhh)/2;

        // ---- 第3步: 删除丢失过久的track ----
        tracks.erase(std::remove_if(tracks.begin(),tracks.end(),
            [](CarTrack&t){return t.lost_frames>MAX_LOST_FRAMES;}),
            tracks.end());

        // ---- 第4步: 检查已匹配track的位置变化(丢失帧跳过) ----
        for(auto&t:tracks){
            if(t.lost_frames>0)continue; // 当前帧未匹配, 保持last_pos不变

            int cx=(t.box.left+t.box.right)/2;
            int cy=(t.box.top+t.box.bottom)/2;
            if(cam_id!=0){cx=xoo+cx*dww/frame.cols;cy=yoo+cy*dhh/frame.rows;}
            int cur_pos=pt_in_any_poly(cx,cy,mask_list)?1:0;

            // 位置变化 → 触发事件(同一ID最多1次驶入+1次驶出)
            if(cur_pos!=t.last_pos){
                if(cur_pos==1&&!t.enter_done){                   // 0→1: 驶入
                    events.push_back({t.id,true});
                    t.enter_done=true;
                }else if(cur_pos==0&&t.enter_done&&!t.exit_done){// 1→0: 驶出(必须先驶入过)
                    events.push_back({t.id,false});
                    t.exit_done=true;
                }
                t.last_pos=cur_pos;
            }
        }

        // ---- 第5步: 为新检测创建track(首次出现不触发事件) ----
        for(int j=0;j<m;j++){
            if(!used[j]){
                CarTrack t;
                t.id=next_id++;
                t.box=dets[j];
                t.lost_frames=0;
                int cx=(dets[j].left+dets[j].right)/2;
                int cy=(dets[j].top+dets[j].bottom)/2;
                if(cam_id!=0){cx=xoo+cx*dww/frame.cols;cy=yoo+cy*dhh/frame.rows;}
                t.last_pos=pt_in_any_poly(cx,cy,mask_list)?1:0;
                // 新ID: 记录位置但不触发事件
                tracks.push_back(t);
            }
        }
    }
};
static void add_event(std::vector<std::string>&log,int chan,bool is_enter,int id){
    time_t now=time(0);struct tm*lt=localtime(&now);
    char buf[128];
    snprintf(buf,128,"%c区 - ID为%d的车辆%s停车场 %04d.%02d.%02d %02d:%02d:%02d",
        'A'+chan,id,is_enter?"驶入":"驶出",
        lt->tm_year+1900,lt->tm_mon+1,lt->tm_mday,
        lt->tm_hour,lt->tm_min,lt->tm_sec);
    log.push_back(buf);if(log.size()>200)log.erase(log.begin());
    system("aplay -q /root/notify.wav &"); // 提示音(后台播放, 不阻塞)
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
    int frame_interval_us[4]={33333,41708,33367,33367}; // A=30 B=23.97 C/D=29.97fps
    int fps_num[4]={30,2397,30000,30000},fps_den[4]={1,100,1001,1001};

    enum{MAIN,VIDEO}state=MAIN;int cam=-1;
    // 按钮: A-F 左侧竖向排列
    int bw=360,bh=100,gap=12,lx=50,ly0=110;
    struct{int x,y;}btn[8];
    for(int i=0;i<8;i++){btn[i].x=lx;btn[i].y=ly0+i*(bh+gap);}
    C4 btn_color=mk(15,15,30), btn_gray=mk(60,60,60);
    // 检测蒙版: 每路摄像头可定义不规则多边形, 只检测区域内目标
    std::vector<std::vector<std::pair<int,int>>> masks[4];
    masks[0]={{{0,343},{726,275},{1278,330},{751,567},{0,434}}}; // A
    masks[1]={{{0,414},{698,295},{1919,455},{1919,1079},{0,1079}}}; // B
    masks[2]={{{481,128},{798,168},{1264,1079},{647,1079}},{{714,0},{1046,246},{1265,305},{1681,561},{1714,424},{920,0}},{{197,59},{340,56},{335,132},{174,136}}}; // C三蒙版
    masks[3]={{{0,483},{1333,438},{1919,879},{1919,1079},{0,1079}}}; // D
    // 停车位蒙版(每通道独立, 仅绘制紫色线条)
    std::vector<std::vector<std::pair<int,int>>> parking_masks[4];
    // A通道停车位
    parking_masks[0]={
        {{0,523},{1094,415},{1193,427},{0,556}},
        {{0,585},{1173,448},{1384,465},{210,653}},
        {{480,652},{578,703},{653,686},{765,744},{1639,509},{1468,472}},
        {{871,808},{1148,834},{1905,506},{1793,495}}
    };
    // D通道停车位
    parking_masks[3]={
        {{116,481},{887,451},{1090,484},{239,522}},
        {{91,557},{419,548},{227,979},{0,987},{0,662}},
        {{640,541},{1057,519},{1290,908},{624,955}},
        {{1294,539},{1448,532},{1883,853},{1633,879}}
    };
    // C通道停车位
    parking_masks[2]={
        {{197,59},{340,56},{335,132},{174,136}},
        {{481,128},{798,168},{1264,1079},{647,1079}},
        {{713,0},{873,0},{1229,224},{1045,244}},
        {{1155,273},{1334,245},{1708,452},{1603,512},{1265,303}}
    };
    // B通道停车位
    parking_masks[1]={
        {{0,414},{698,295},{870,315},{0,494}},
        {{0,568},{974,357},{1119,399},{0,728}},
        {{596,658},{1312,429},{1586,444},{1178,686}},
        {{1390,807},{1725,963},{1918,636},{1916,458}}
    };
    cv::Mat frame,fb0,fb1,fb2,fb3;
    {cv::Mat t;cap0.read(t);if(!t.empty())t.copyTo(fb0);
     cap1.read(t);if(!t.empty())t.copyTo(fb1);
     cap2.read(t);if(!t.empty())t.copyTo(fb2);
     cap3.read(t);if(!t.empty())t.copyTo(fb3);}
    // 绝对时钟: 四路视频启动时间
    std::chrono::steady_clock::time_point video_start[4];
    int64_t frame_cnt[4]={};
    {auto now0=std::chrono::steady_clock::now();
     for(int i=0;i<4;i++){video_start[i]=now0;frame_cnt[i]=1;}}
    printf("[GUI] started, %dx%d\n",drm.w,drm.h);fflush(stdout);

    int empty_cnt=0, det_skip=0, fps_cnt=0, time_skip=0;
    char tb_time[32]={}, tb_date[32]={};
    {time_t now=time(0);struct tm*lt=localtime(&now);
     strftime(tb_time,32,"%H:%M:%S",lt);strftime(tb_date,32,"%Y.%m.%d",lt);}
    std::vector<DetectBox> last_boxes;
    char fps_str[16]={"FPS: --"}, det_str[16]={"NPU: --ms"}, car_str[16]={"CAR:0"}, per_str[16]={"PER:0"};
    int det_ms=0, cnt_car_acc=0, cnt_per_acc=0, avg_cnt=0;
    int chan_car[4]={0}, chan_per[4]={0}, chan_cap[4]={200,120,24,40}; // 每通道车/人/容量
    char chan_info[4][32]={}, chan_status[4][16]={};
    int bg_det_skip[4]={};
    bool chan_enabled[8]={true,true,true,true,true,true,true,true};
    std::vector<std::string> event_log;
    int cb_x0=450,cb_y0=105,cb_btn=54,cb_gap=6;
    int log_x=450,log_y=175,log_w=1440,log_h=880,log_line_h=30;
    static int bg_sel=0; static bool bg_done=false;
    CarTracker car_trackers[4];
    auto fps_t0=std::chrono::steady_clock::now();

    while(1){
        touch.poll();int tx,ty;touch.landscape(tx,ty);

        // 后台四路同时原速播放: 绝对时钟驱动, 落后时跳帧追赶
        {for(int c=0;c<4;c++){
            if(state==VIDEO){
                if(c==cam)continue;
                if(bg_done)continue;
                int picked=-1, nc=0;
                for(int x=0;x<4;x++)if(x!=cam){if(nc==bg_sel)picked=x;nc++;}
                if(c!=picked)continue;
                bg_done=true; bg_sel=(bg_sel+1)%nc;
            }
            auto now_clk=std::chrono::steady_clock::now();
            int64_t elaps=std::chrono::duration_cast<std::chrono::microseconds>(now_clk-video_start[c]).count();
            int target=(int)(elaps*fps_num[c]/fps_den[c]/1000000);
            // 跳帧追赶(最多60帧)
            cv::Mat t; bool rok=false;
            if(frame_cnt[c]<target){
                cv::Mat skip; int smax=target;
                if(smax-frame_cnt[c]>60)smax=frame_cnt[c]+60;
                while(frame_cnt[c]<smax){
                    if(c==0)rok=cap0.read(skip);else if(c==1)rok=cap1.read(skip);
                    else if(c==2)rok=cap2.read(skip);else rok=cap3.read(skip);
                    if(!rok)break; frame_cnt[c]++;
                }
            }
            // 读当前帧
            if(c==0)rok=cap0.read(t);else if(c==1)rok=cap1.read(t);
            else if(c==2)rok=cap2.read(t);else rok=cap3.read(t);
            if(!rok||t.empty())continue;
            frame_cnt[c]++;
            // 存入缓冲帧
            if(c==0)t.copyTo(fb0);else if(c==1)t.copyTo(fb1);else if(c==2)t.copyTo(fb2);else t.copyTo(fb3);
            // 检测: 每4轮检一次(降低CPU占用)
            if(!masks[c].empty()){
                bg_det_skip[c]=(bg_det_skip[c]+1)%2;
                if(bg_det_skip[c]==0){
                    std::vector<DetectBox> bbx;det.detect(t,bbx);
                    int dwb=1920,dhb=t.rows*1920/t.cols;
                    if(dhb>1080){dhb=1080;dwb=t.cols*1080/t.rows;}
                    int xob=(1920-dwb)/2,yob=(1080-dhb)/2;
                    int cc=0,pc=0;for(auto&b:bbx){int cxx=(b.left+b.right)/2,cyy=(b.top+b.bottom)/2;
                        if(c!=0){cxx=xob+cxx*dwb/t.cols;cyy=yob+cyy*dhb/t.rows;}
                        if(pt_in_any_poly(cxx,cyy,masks[c])){if(is_v(b.class_id))cc++;else if(b.class_id==0)pc++;}
                    }
                    chan_per[c]=pc;
                    std::vector<DetectBox> veh;for(auto&b:bbx)if(is_v(b.class_id))veh.push_back(b);
                    std::vector<CarTracker::Event> evs;
                    car_trackers[c].update(veh,masks[c],t,c,evs);
                    for(auto&ev:evs)if(chan_enabled[c])add_event(event_log,c,ev.is_enter,ev.id);
                    chan_car[c]=cc;
                    snprintf(chan_info[c],32,"数量:%d/%d",cc,chan_cap[c]);
                    {float r=(float)cc/chan_cap[c];snprintf(chan_status[c],16,"状态:%s",r<0.3f?"畅通":r<0.7f?"中等":"拥挤");}
                }
            }
        }}
        bg_done=false;

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
            for(int i=0;i<8;i++){
                C4 clr=(i<4)?btn_color:btn_gray;
                drm.fL(bk,btn[i].x,btn[i].y,bw,bh,clr);
                drm.bL(bk,btn[i].x,btn[i].y,bw,bh,mk(255,255,255),3);
                char big[2]={(char)('A'+i),0};
                C4 tc=(i<4)?mk(255,255,255):mk(120,120,120);
                int cx=btn[i].x+120, cy=btn[i].y+10;
                ft_text(drm,bk,cx-ft_width(big,80)/2,cy+20,big,80,tc);
                if(i<4 && chan_info[i][0]){
                    float r=(float)chan_car[i]/chan_cap[i];
                    C4 sc=r<0.3f?mk(0,255,0):r<0.7f?mk(255,165,0):mk(255,0,0);
                    ft_text(drm,bk,btn[i].x+240,btn[i].y+bh-50,chan_status[i],20,sc);
                    ft_text(drm,bk,btn[i].x+240,btn[i].y+bh-20,chan_info[i],20,tc);
                }
            }
            // === 通道按钮 A-H (一行) ===
            for(int i=0;i<8;i++){
                int cx=cb_x0+i*120, cy=cb_y0;
                drm.fL(bk,cx,cy,cb_btn,cb_btn,chan_enabled[i]?mk(30,80,30):mk(50,50,50));
                drm.bL(bk,cx,cy,cb_btn,cb_btn,chan_enabled[i]?mk(0,200,0):mk(100,100,100),2);
                char lb[2]={char('A'+i),0};
                C4 tc=chan_enabled[i]?mk(0,255,0):mk(130,130,130);
                ft_text(drm,bk,cx+cb_btn/2+15,cy+cb_btn/2-14,lb,44,tc);
            }
            // === 日志框(无滚动, 最新在上) ===
            drm.fL(bk,log_x,log_y,log_w,log_h,mk(5,5,15));
            drm.bL(bk,log_x,log_y,log_w,log_h,mk(60,60,60));
            {int vis=log_h/log_line_h;int shown=0;
             for(int i=0;shown<vis&&i<(int)event_log.size();i++){
                int ei=(int)event_log.size()-1-i;
                const char*s=event_log[ei].c_str();
                if(!chan_enabled[s[0]-'A']){continue;} // 跳过已取消通道
                ft_text(drm,bk,480,log_y+15+shown*log_line_h,s,24,mk(200,200,200));
                shown++;
             }}
            drm.flip();
            // === 触摸: 通道按钮切换 ===
            static bool was_down=false;
            if(touch.down){
                if(!was_down){for(int i=0;i<8;i++){
                    int cx=cb_x0+i*120, cy=cb_y0;
                    if(tx>=cx&&tx<cx+cb_btn&&ty>=cy&&ty<cy+cb_btn){chan_enabled[i]=!chan_enabled[i];break;}
                }was_down=true;}
            }else{was_down=false;}
            if(touch.clicked){for(int i=0;i<4;i++)if(tx>=btn[i].x&&tx<btn[i].x+bw&&ty>=btn[i].y&&ty<btn[i].y+bh){
                cam=i;state=VIDEO;empty_cnt=0;
                cnt_car_acc=cnt_per_acc=avg_cnt=0;
                snprintf(car_str,16,"CAR:%d",chan_car[cam]);snprintf(per_str,16,"PER:%d",chan_per[cam]);
                // 视频时钟已在后台运行, 直接显示缓冲帧
                if(cam==0)fb0.copyTo(frame);else if(cam==1)fb1.copyTo(frame);
                else if(cam==2)fb2.copyTo(frame);else fb3.copyTo(frame);
                {int bk2=drm.fr^1;drm.clear(bk2);
                 drm.rot_write(bk2,frame,1920,1080);
                 drm.fL(bk2,20,20,90,50,mk(50,0,0));drm.bL(bk2,20,20,90,50,mk(200,100,100));
                 ft_text(drm,bk2,85-ft_width("返回",22)/2,38,"返回",22,mk(255,200,200));
                 drm.flip();}
                break;}}
            usleep(2000);
        }else{
            // 绝对时钟: 计算该播第几帧, 落后则跳帧追赶
            auto now_clk=std::chrono::steady_clock::now();
            int64_t elaps=std::chrono::duration_cast<std::chrono::microseconds>(now_clk-video_start[cam]).count();
            int target=(int)(elaps*fps_num[cam]/fps_den[cam]/1000000);
            // 落后超过1帧: 快速读帧丢弃(不渲染), 限制最多60帧
            cv::Mat f; bool ok=false;
            if(frame_cnt[cam]<target){
                cv::Mat skip; int smax=target;
                if(smax-frame_cnt[cam]>60)smax=frame_cnt[cam]+60;
                while(frame_cnt[cam]<smax){
                    if(cam==0)ok=cap0.read(skip);else if(cam==1)ok=cap1.read(skip);
                    else if(cam==2)ok=cap2.read(skip);else ok=cap3.read(skip);
                    if(!ok)break; frame_cnt[cam]++;
                }
            }
            // 读当前帧
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
            frame_cnt[cam]++;
            f.copyTo(frame);
            // NPU检测跳帧: 每3帧更新一次框位置, 但每帧都画(消除闪烁)
            det_skip=(det_skip+1)%6;
            if(det_skip==0){last_boxes.clear();auto t1=std::chrono::steady_clock::now();det.detect(frame,last_boxes);auto t2=std::chrono::steady_clock::now();det_ms=(int)std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();snprintf(det_str,16,"NPU: %dms",det_ms);
                if(!masks[cam].empty()){
                    // 车辆进出跟踪(全部车辆检测, 非仅蒙版内)
                    {std::vector<DetectBox> veh;for(auto&b:last_boxes)if(is_v(b.class_id))veh.push_back(b);
                     std::vector<CarTracker::Event> evs;
                     car_trackers[cam].update(veh,masks[cam],frame,cam,evs);
                     for(auto&ev:evs)if(chan_enabled[cam])add_event(event_log,cam,ev.is_enter,ev.id);
                     }
                    // 蒙版过滤(仅显示蒙版内目标)
                    std::vector<DetectBox> filtered;
                    for(auto&b:last_boxes){
                        int cx=(b.left+b.right)/2,cy=(b.top+b.bottom)/2;
                        if(cam!=0){ // B/C/D: 帧→画布坐标转换
                            int dww=1920,dhh=frame.rows*1920/frame.cols;
                            if(dhh>1080){dhh=1080;dww=frame.cols*1080/frame.rows;}
                            int xoo=(1920-dww)/2,yoo=(1080-dhh)/2;
                            cx=xoo+cx*dww/frame.cols;cy=yoo+cy*dhh/frame.rows;
                        } // A(cam==0): 保持简单对比
                        if(pt_in_any_poly(cx,cy,masks[cam]))filtered.push_back(b);
                    }
                    last_boxes=filtered;
                }
            }
            // 统计蒙版内车/人数量, 累加10帧取平均
            {int cnt_car=0,cnt_per=0;
            for(auto&b:last_boxes){if(b.class_id==0)cnt_per++;else if(is_v(b.class_id))cnt_car++;}
            cnt_car_acc+=cnt_car;cnt_per_acc+=cnt_per;avg_cnt++;
            if(avg_cnt>=30){
                int cc=(int)(cnt_car_acc/30.0+0.5);
                snprintf(car_str,16,"CAR:%d",cc);
                snprintf(per_str,16,"PER:%d",(cnt_per_acc+29)/30);
                chan_car[cam]=cc;
                snprintf(chan_info[cam],32,"数量:%d/%d",cc,chan_cap[cam]);
                {float r=(float)cc/chan_cap[cam];snprintf(chan_status[cam],16,"%s",r<0.3f?"畅通":r<0.7f?"中等":"拥挤");}
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
            if(cam==0&&!masks[0].empty())draw_mask_A(drm,bk,masks[0][0]);
            else if(!masks[cam].empty())draw_all_polys(drm,bk,masks[cam],mk(255,255,0));
            if(!parking_masks[cam].empty()){
                if(cam==0){for(auto&p:parking_masks[0])draw_poly(drm,bk,p,mk(255,0,255));}
                else draw_all_polys(drm,bk,parking_masks[cam],mk(255,0,255));
            }
            // 返回按钮 — 文字居中
            drm.fL(bk,20,20,90,50,mk(50,0,0));drm.bL(bk,20,20,90,50,mk(200,100,100));
            ft_text(drm,bk,85-ft_width("返回",22)/2,38,"返回",22,mk(255,200,200));
            drm.flip();
            // FPS统计 (每2秒输出)
            fps_cnt++;
            auto fps_now=std::chrono::steady_clock::now();
            int fps_ms=std::chrono::duration_cast<std::chrono::milliseconds>(fps_now-fps_t0).count();
            if(fps_ms>=2000){double fps=fps_cnt*1000.0/fps_ms;snprintf(fps_str,16,"FPS: %.0f",fps);fps_cnt=0;fps_t0=fps_now;}
            if(touch.clicked){if(tx>=20&&tx<=110&&ty>=20&&ty<=70){state=MAIN;cam=-1;empty_cnt=0;}}
            // 帧定时: 绝对时钟已保证时速, 小sleep防busy-loop
            if(state==VIDEO)usleep(500);
        }}
    det.release();return 0;
}
