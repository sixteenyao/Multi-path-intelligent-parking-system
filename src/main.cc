/**
 * smart_camera — 主入口
 * 图片: ./smart_camera yolo11s.rknn bus.jpg
 * 视频: ./smart_camera yolo11s.rknn videos/a.mp4
 */
#include <iostream>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <opencv2/opencv.hpp>
#include "detector.h"
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm_fourcc.h>

// ---- 手写字体 ----
static unsigned char font[128][35];
static bool font_ok = false;
static void init_font() {
    if (font_ok) return; font_ok = true;
    memset(font, 0, sizeof(font));
    auto def = [](char ch, const char* s) {
        for (int i = 0; i < 35; i++) font[(int)ch][i] = (s[i]=='1');
    };
    def('0',"0111010001100011000110001100011000101110");
    def('1',"0010001100001000010000100001000010001110");
    def('2',"0111010001000010010000100001000010001111");
    def('3',"0111010001000010011000001100011000101110");
    def('4',"0001000110010101001011110001000010000100");
    def('5',"1111110000100001111000001000011000101110");
    def('6',"0111010001100001111010001100011000101110");
    def('7',"1111100001000100001000100001000010000100");
    def('8',"0111010001100010111010001100011000101110");
    def('9',"0111010001100011000101111000011000101110");
    def('A',"01110100011000111111100011000110001");
    def('B',"11110100011000111110100011000111110");
    def('C',"01110100011000010000100001000101110");
    def('E',"11111100001000011110100001000011111");
    def('F',"11111100001000011110100001000010000");
    def('G',"01110100011000010111100011000101110");
    def('H',"10001100011000111111100011000110001");
    def('I',"01110001000010000100001000010001110");
    def('K',"10001100101010011000101001001010001");
    def('L',"10000100001000010000100001000011111");
    def('M',"10001110111010110101100011000110001");
    def('N',"10001110011010110011100011000110001");
    def('O',"01110100011000110001100011000101110");
    def('P',"11110100011000111110100001000010000");
    def('R',"11110100011000111110100011000110001");
    def('S',"01110100011000001110000011000101110");
    def('T',"11111001000010000100001000010000100");
    def('U',"10001100011000110001100011000101110");
    def('V',"10001100011000101010010100010000100");
    def('W',"10001100011000110101101011010101010");
    def('X',"10001100010101000100010101000110001");
    def('Y',"10001100010101000100001000010000100");
    def('%',"00001000000010000100001001000010000");
}
static void draw_char(cv::Mat& m, int cx, int cy, char ch, uchar B, uchar G, uchar R) {
    uchar* f = font[(int)ch];
    for (int fy=0; fy<7; fy++) for (int fx=0; fx<5; fx++)
        if (f[fy*5+fx])
            for (int dy=0; dy<2; dy++) for (int dx=0; dx<2; dx++) {
                int py=cy+fy*2+dy, px=cx+fx*2+dx;
                if (py>=0&&py<m.rows&&px>=0&&px<m.cols) {
                    auto* p=m.ptr(py)+px*3; p[0]=B;p[1]=G;p[2]=R;
                }
            }
}
static void draw_label(cv::Mat& f, int l, int t, int r, int btm, int cls, float conf) {
    const char* sn="??";
    if (cls==0) sn="PER"; else if (cls==5) sn="BUS"; else if (cls==2) sn="CAR";
    int ly=t-20; if (ly<0) ly=btm+2;
    int lbl_w=120, lx=l; if (lx+lbl_w>f.cols) lx=f.cols-lbl_w;
    for (int y=ly; y<ly+16&&y<f.rows; y++) for (int x=lx; x<lx+lbl_w&&x<f.cols; x++) {
        auto* p=f.ptr(y)+x*3; p[0]/=2;p[1]/=2;p[2]/=2;
    }
    int cx=lx+3;
    for (const char* s=sn; *s; s++, cx+=12) draw_char(f,cx,ly+2,*s,0,255,255);
    char pct[8]; snprintf(pct,8,"%d%%",(int)(conf*100));
    cx=lx+50;
    for (const char* s=pct; *s; s++, cx+=12) draw_char(f,cx,ly+2,*s,(*s>='0'&&*s<='9')?0:255,(*s>='0'&&*s<='9')?255:255,0);
}

// ---- DRM 显示 ----
struct DRM { int fd=0, front=0; uint32_t fb_id[2]={}, crtc, conn, w, h, pitch, size; drmModeModeInfo mode; unsigned char* buf[2]={}; };
static bool drm_init(DRM& d) {
    d.fd=open("/dev/dri/card0",O_RDWR); if(d.fd<0) return false;
    auto* res=drmModeGetResources(d.fd);
    for(int i=0;i<res->count_connectors;i++){auto*c=drmModeGetConnector(d.fd,res->connectors[i]); if(c&&c->connection==DRM_MODE_CONNECTED&&c->count_modes>0){d.conn=c->connector_id;d.mode=c->modes[0];drmModeEncoder*e=drmModeGetEncoder(d.fd,c->encoder_id);if(e){d.crtc=e->crtc_id;drmModeFreeEncoder(e);}drmModeFreeConnector(c);break;}drmModeFreeConnector(c);}
    drmModeFreeResources(res);
    d.w=d.mode.hdisplay; d.h=d.mode.vdisplay;
    for(int b=0; b<2; b++) {
        struct drm_mode_create_dumb c={}; c.width=d.w;c.height=d.h;c.bpp=32; drmIoctl(d.fd,DRM_IOCTL_MODE_CREATE_DUMB,&c);
        if(b==0){d.pitch=c.pitch; d.size=c.size;}
        uint32_t h[4]={(uint32_t)c.handle,0u,0u,0u}, p[4]={(uint32_t)c.pitch,0u,0u,0u}, o[4]={0u,0u,0u,0u};
        drmModeAddFB2(d.fd,d.w,d.h,DRM_FORMAT_XRGB8888,h,p,o,&d.fb_id[b],0);
        struct drm_mode_map_dumb m={}; m.handle=c.handle; drmIoctl(d.fd,DRM_IOCTL_MODE_MAP_DUMB,&m);
        d.buf[b]=(unsigned char*)mmap(0,d.size,PROT_READ|PROT_WRITE,MAP_SHARED,d.fd,m.offset);
    }
    drmModeSetCrtc(d.fd,d.crtc,d.fb_id[0],0,0,&d.conn,1,&d.mode);  // 初始显示buf[0]
    return d.buf[0]!=MAP_FAILED&&d.buf[1]!=MAP_FAILED;
}
static void drm_show_frame(DRM& d, cv::Mat& img) {
    int back = d.front ^ 1;
    // 横屏模式: 先将图片缩放到 1920×1080, 再旋转90°写入 1080×1920
    int lw=1920, lh=1080;  // 横屏逻辑尺寸
    int dw=lw, dh=img.rows*lw/img.cols;
    if(dh>lh){dh=lh;dw=img.cols*lh/img.rows;}
    int xo=(lw-dw)/2, yo=(lh-dh)/2;

    for(int y=0;y<d.h;y++)memset(d.buf[back]+y*d.pitch,0,d.w*4);

    for(int y=0;y<dh;y++){int sy=y*img.rows/dh;
        for(int x=0;x<dw;x++){int sx=x*img.cols/dw;
            // 横屏→顺时针270°(原90°+再180°)→竖屏: (lx,ly)→(1079-ly, lx)
            int lx=xo+x, ly=yo+y;
            int px=lh-1-ly, py=lx;
            if(px>=0&&px<d.w&&py>=0&&py<d.h){
                auto* row=d.buf[back]+py*d.pitch+px*4;
                auto* src=img.ptr(sy)+sx*3;
                row[0]=src[0]; row[1]=src[1]; row[2]=src[2]; row[3]=0;
            }
        }
    }
    drmModeSetCrtc(d.fd,d.crtc,d.fb_id[back],0,0,&d.conn,1,&d.mode);
    d.front = back;
}
static void drm_close(DRM& d){drmModeSetCrtc(d.fd,d.crtc,0,0,0,NULL,0,NULL);for(int b=0;b<2;b++){munmap(d.buf[b],d.size);drmModeRmFB(d.fd,d.fb_id[b]);}close(d.fd);}

// ---- 画框到像素 ----
static bool is_vehicle(int cls){return cls==1||cls==2||cls==3||cls==5||cls==6||cls==7;}
static void draw_boxes(cv::Mat& f, const std::vector<DetectBox>& boxes) {
    const int T=4;
    for(const auto& b:boxes){int l=std::max(0,(int)b.left),t=std::max(0,(int)b.top),r=std::min(f.cols-1,(int)b.right),btm=std::min(f.rows-1,(int)b.bottom);
        unsigned char B,G,R;
        if(b.class_id==0){B=150;G=0;R=255;}
        else if(is_vehicle(b.class_id)){B=255;G=80;R=0;}
        else{B=0;G=255;R=0;}
        for(int y=t;y<=btm;y++){auto*row=f.ptr(y);bool te=(y-t<T),be=(btm-y<T);
            for(int x=l;x<=r;x++){bool le=(x-l<T),re=(r-x<T);if((te||be)||(le||re)){row[x*3]=B;row[x*3+1]=G;row[x*3+2]=R;}}}
        draw_label(f,l,t,r,btm,b.class_id,b.confidence);}
}

// ---- 主函数 ----
int main(int argc, char** argv) {
    // 杀 Weston 释放 DRM
    system("pkill -9 weston 2>/dev/null"); usleep(600000);
    const char* model=(argc>1)?argv[1]:"yolo11s.rknn";
    const char* input=(argc>2)?argv[2]:"bus.jpg";

    init_font();
    Detector det; det.init(model,0);

    cv::VideoCapture cap;
    cv::Mat frame;
    bool is_video = false;

    // 尝试作为视频打开
    if (cap.open(input)) {
        is_video = true;
        std::cout << "[main] 视频模式: " << input << std::endl;
    } else {
        frame = cv::imread(input);
        if (frame.empty()) { std::cerr<<"无法打开: "<<input<<std::endl; return -1; }
        std::cout<<"[main] 图片模式: "<<input<<" "<<frame.cols<<"x"<<frame.rows<<std::endl;
    }

    DRM drm;
    if (!drm_init(drm)) { std::cerr<<"DRM失败"<<std::endl; return -1; }

    int total=0;
    int64_t total_ms=0;
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat display;

    while (true) {
        if (is_video) {
            if (!cap.read(frame) || frame.empty()) { std::cout<<"视频结束\n"; break; }
            // detector 内部做 letterbox，不需要预缩放
        }

        std::vector<DetectBox> boxes;
        auto t1 = std::chrono::steady_clock::now();
        det.detect(frame, boxes);
        auto t2 = std::chrono::steady_clock::now();
        int ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        total++; total_ms+=ms;

        draw_boxes(frame, boxes);
        drm_show_frame(drm, frame);

        // FPS统计
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count();
        if (elapsed >= 5000) {
            std::cout << "[FPS] " << total << " frames, avg " << (total_ms/total) << "ms, "
                      << (total*1000/elapsed) << " FPS" << std::endl;
            t0 = std::chrono::steady_clock::now();
            total = total_ms = 0;
        }

        if (!is_video) { sleep(10); break; }
    }

    drm_close(drm);
    det.release();
    return 0;
}
