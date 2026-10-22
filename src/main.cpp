#include "pathtracer.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <cmath>
#include <memory>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#ifdef MPT_WITH_SDL2
#include <SDL.h>
#endif

namespace {
struct Cli { mpt::Settings settings; std::string out="cornell.bmp"; bool headless=false; };
void help(){
    std::cout<<"MiniPathTracer - progressive CPU Cornell Box path tracer\n"
      <<"  --width N       image width (default 480)\n"
      <<"  --height N      image height (default 480)\n"
      <<"  --spp N         samples per pixel (default 128)\n"
      <<"  --bounces N     max light bounces (default 8)\n"
      <<"  --threads N     worker threads (0 = auto)\n"
      <<"  --exposure X    tone mapping exposure (default 1.0)\n"
      <<"  --seed N        deterministic random seed (default 42)\n"
      <<"  --light-x X     area-light X offset in scene units (default 0)\n"
      <<"  --light-y Y     area-light Y offset (down is negative)\n"
      <<"  --light-z Z     area-light Z offset (default 0)\n"
      <<"  --out FILE      output image (BMP or PPM)\n"
      <<"  --headless      no SDL2 preview (always works)\n"
      <<"  --help          show this text\n\n"
      <<"SDL2: drag orbit, WASD/QE camera; arrows move lamp X/Z, PgUp/PgDn lamp Y, Home reset lamp; [ ] brightness, F12 save.\n";
}
Cli parse(int argc,char** argv){
    Cli opt;
    auto take=[&](int& i){if(i+1>=argc)throw std::invalid_argument("Missing value for "+std::string(argv[i]));return std::string(argv[++i]);};
    for(int i=1;i<argc;++i){std::string a=argv[i];
        if(a=="--width")opt.settings.width=std::stoi(take(i));
        else if(a=="--height")opt.settings.height=std::stoi(take(i));
        else if(a=="--spp")opt.settings.samples=std::stoi(take(i));
        else if(a=="--bounces")opt.settings.max_bounces=std::stoi(take(i));
        else if(a=="--threads")opt.settings.threads=std::stoi(take(i));
        else if(a=="--exposure")opt.settings.exposure=std::stof(take(i));
        else if(a=="--seed")opt.settings.seed=std::stoull(take(i));
        else if(a=="--light-x")opt.settings.light_offset.x=std::stof(take(i));
        else if(a=="--light-y")opt.settings.light_offset.y=std::stof(take(i));
        else if(a=="--light-z")opt.settings.light_offset.z=std::stof(take(i));
        else if(a=="--out")opt.out=take(i);
        else if(a=="--headless")opt.headless=true;
        else if(a=="--help"||a=="-h"){help();std::exit(0);}
        else throw std::invalid_argument("Unknown option: "+a);
    }
    auto& s=opt.settings;
    if(s.width<1||s.height<1||s.width>4096||s.height>4096||s.samples<1||s.samples>100000||s.max_bounces<1||s.max_bounces>64||s.threads<0||s.threads>256||s.exposure<=0||
       !std::isfinite(s.light_offset.x)||!std::isfinite(s.light_offset.y)||!std::isfinite(s.light_offset.z))
        throw std::invalid_argument("Invalid settings (dimensions 1..4096, spp 1..100000, bounces 1..64, threads 0..256, exposure > 0)");
    return opt;
}
void log_pass(int sample,const mpt::Settings& s,std::chrono::steady_clock::time_point start){
    if(sample==s.samples||sample==1||sample%std::max(1,s.samples/10)==0){
      auto seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::cout<<"Samples "<<sample<<"/"<<s.samples<<"  ("<<seconds<<" s)\n"<<std::flush;
    }
}
void headless(mpt::Renderer& renderer,const Cli& opt){
    mpt::Image image(opt.settings.width,opt.settings.height);
    auto start=std::chrono::steady_clock::now();
    renderer.render(opt.settings,image,[&](int sample,const mpt::Image&){log_pass(sample,opt.settings,start);});
    if(!image.save(opt.out,opt.settings.exposure))throw std::runtime_error("Could not save "+opt.out);
    std::cout<<"Saved "<<opt.out<<"\n";
}
#ifdef MPT_WITH_SDL2
void preview(mpt::Renderer& renderer,const Cli& opt){
    if(SDL_Init(SDL_INIT_VIDEO)!=0)throw std::runtime_error(SDL_GetError());
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY,"1");
    SDL_Window* window=SDL_CreateWindow("MiniPathTracer | interactive preview",
        SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
        std::max(480,opt.settings.width),std::max(480,opt.settings.height),SDL_WINDOW_RESIZABLE);
    if(!window){SDL_Quit();throw std::runtime_error(SDL_GetError());}
    SDL_Renderer* display=SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    if(!display) display=SDL_CreateRenderer(window,-1,SDL_RENDERER_SOFTWARE);
    if(!display){SDL_DestroyWindow(window);SDL_Quit();throw std::runtime_error(SDL_GetError());}
    SDL_Texture* texture=nullptr;
    int tex_w=0,tex_h=0;
    // The camera orbits around a movable target. No OpenGL lighting is involved:
    // SDL only displays pixels computed by the path tracer on CPU threads.
    float yaw=0.0f,pitch=0.0f,distance=3.35f;
    mpt::Vec3 focus{0,1,0};
    float light_scale=opt.settings.light_scale;
    mpt::Vec3 light_offset=opt.settings.light_offset;
    int final_spp=opt.settings.samples;
    std::atomic<float> exposure{opt.settings.exposure};
    std::atomic<bool> cancel{false};
    std::thread job;
    std::unique_ptr<mpt::Image> render_image;
    std::mutex frame_mutex;
    std::vector<uint8_t> pending_rgb,current_rgb;
    int pending_w=0,pending_h=0,pending_spp=0;
    int rendered_spp=0;
    bool coarse_active=false;
    auto start_render=[&](bool coarse){
        cancel.store(true);
        if(job.joinable())job.join();
        cancel.store(false);
        mpt::Settings settings=opt.settings;
        settings.samples=final_spp;
        settings.light_scale=light_scale;
        settings.light_offset=light_offset;
        const float cp=std::cos(pitch);
        settings.camera_target=focus;
        settings.camera_position=focus+mpt::Vec3{
            distance*cp*std::sin(yaw),distance*std::sin(pitch),distance*cp*std::cos(yaw)};
        if(coarse){
            const int longer=std::max(settings.width,settings.height);
            const float factor=std::min(1.0f,160.0f/static_cast<float>(longer));
            settings.width=std::max(1,static_cast<int>(settings.width*factor));
            settings.height=std::max(1,static_cast<int>(settings.height*factor));
            settings.samples=1;
            settings.max_bounces=std::min(settings.max_bounces,4);
        }
        coarse_active=coarse;
        render_image.reset(new mpt::Image(settings.width,settings.height));
        mpt::Image* target=render_image.get();
        job=std::thread([&,settings,target]{
            renderer.render(settings,*target,[&](int spp,const mpt::Image& image){
                auto rgb=image.rgb8(exposure.load());
                std::lock_guard<std::mutex> lock(frame_mutex);
                pending_rgb=std::move(rgb);
                pending_w=settings.width;
                pending_h=settings.height;
                pending_spp=spp;
            },&cancel);
        });
    };
    auto save_screenshot=[&]{
        if(current_rgb.empty()){
            std::cout<<"No completed preview frame to save yet.\n";
            return;
        }
        SDL_Surface* surface=SDL_CreateRGBSurfaceWithFormatFrom(current_rgb.data(),
             tex_w,tex_h,24,tex_w*3,SDL_PIXELFORMAT_RGB24);
        if(!surface){std::cerr<<"Screenshot error: "<<SDL_GetError()<<"\n";return;}
        // SDL_SaveBMP always writes BMP, irrespective of the provided extension.
        std::string out=opt.out;
        if(out.size()<4 || out.substr(out.size()-4)!=".bmp")out="cornell_live.bmp";
        if(SDL_SaveBMP(surface,out.c_str())==0)std::cout<<"Saved "<<out<<"\n";
        else std::cerr<<"Screenshot error: "<<SDL_GetError()<<"\n";
        SDL_FreeSurface(surface);
    };
    Uint32 last_restart=SDL_GetTicks()-250;
    Uint32 last_interaction=SDL_GetTicks();
    Uint32 last_tick=SDL_GetTicks();
    bool dirty=false,running=true;
    start_render(true);
    last_restart=SDL_GetTicks();
    while(running){
        bool moved=false;
        SDL_Event event;
        while(SDL_PollEvent(&event)){
            if(event.type==SDL_QUIT)running=false;
            if(event.type==SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON_LMASK)){
                yaw-=0.007f*event.motion.xrel;
                pitch=std::clamp(pitch-0.007f*event.motion.yrel,-1.15f,1.15f);
                moved=true;
            }
            if(event.type==SDL_MOUSEWHEEL){
                distance=std::clamp(distance*std::pow(0.9f,static_cast<float>(event.wheel.y)),0.35f,15.0f);
                moved=true;
            }
            if(event.type==SDL_KEYDOWN && event.key.repeat==0){
                SDL_Keycode k=event.key.keysym.sym;
                if(k==SDLK_ESCAPE)running=false;
                if(k==SDLK_F12 || (k==SDLK_s && (event.key.keysym.mod & KMOD_CTRL)))save_screenshot();
                if(k==SDLK_r){yaw=0;pitch=0;distance=3.35f;focus={0,1,0};moved=true;}
                if(k==SDLK_HOME){light_offset={0,0,0};moved=true;}
                if(k==SDLK_LEFTBRACKET){light_scale=std::max(0.02f,light_scale/1.25f);moved=true;}
                if(k==SDLK_RIGHTBRACKET){light_scale=std::min(20.f,light_scale*1.25f);moved=true;}
                if(k==SDLK_1){final_spp=16;moved=true;}
                if(k==SDLK_2){final_spp=64;moved=true;}
                if(k==SDLK_3){final_spp=256;moved=true;}
                if(k==SDLK_EQUALS || k==SDLK_PLUS || k==SDLK_KP_PLUS){exposure.store(exposure.load()*1.2f);moved=true;}
                if(k==SDLK_MINUS || k==SDLK_KP_MINUS){exposure.store(exposure.load()/1.2f);moved=true;}
            }
        }
        Uint32 now=SDL_GetTicks();
        float dt=std::min(0.05f,(now-last_tick)/1000.0f);
        last_tick=now;
        const Uint8* keys=SDL_GetKeyboardState(nullptr);
        mpt::Vec3 dir{std::sin(yaw),0,std::cos(yaw)};
        mpt::Vec3 right{std::cos(yaw),0,-std::sin(yaw)};
        const float velocity=1.2f*dt;
        if(keys[SDL_SCANCODE_W]){focus=focus-dir*velocity;moved=true;}
        if(keys[SDL_SCANCODE_S] && !(SDL_GetModState()&KMOD_CTRL)){focus=focus+dir*velocity;moved=true;}
        if(keys[SDL_SCANCODE_A]){focus=focus-right*velocity;moved=true;}
        if(keys[SDL_SCANCODE_D]){focus=focus+right*velocity;moved=true;}
        if(keys[SDL_SCANCODE_Q]){focus.y-=velocity;moved=true;}
        if(keys[SDL_SCANCODE_E]){focus.y+=velocity;moved=true;}
        // Move the actual emitting quad, not only the point used for shadow sampling.
        // Keep the 0.56x0.56 lamp inside the Cornell Box and above the tall box.
        const float light_velocity=0.85f*dt;
        mpt::Vec3 next_light=light_offset;
        if(keys[SDL_SCANCODE_LEFT])next_light.x-=light_velocity;
        if(keys[SDL_SCANCODE_RIGHT])next_light.x+=light_velocity;
        if(keys[SDL_SCANCODE_UP])next_light.z-=light_velocity;
        if(keys[SDL_SCANCODE_DOWN])next_light.z+=light_velocity;
        if(keys[SDL_SCANCODE_PAGEUP])next_light.y+=light_velocity;
        if(keys[SDL_SCANCODE_PAGEDOWN])next_light.y-=light_velocity;
        next_light.x=std::clamp(next_light.x,-0.65f,0.65f);
        next_light.y=std::clamp(next_light.y,-0.55f,0.0f);
        next_light.z=std::clamp(next_light.z,-0.30f,1.10f);
        if(mpt::length2(next_light-light_offset)>1e-12f){
            light_offset=next_light;
            moved=true;
        }
        if(moved){dirty=true;last_interaction=now;}
        // While interacting, rapidly replace the preview at low resolution.
        // After 220 ms idle, switch to full-resolution progressive sampling.
        if(dirty && now-last_restart>=110){
            start_render(true);dirty=false;last_restart=SDL_GetTicks();
        }else if(!dirty && coarse_active && now-last_interaction>=220 && now-last_restart>=100){
            start_render(false);last_restart=SDL_GetTicks();
        }
        std::vector<uint8_t> received;
        int w=0,h=0,spp=0;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if(!pending_rgb.empty()){
                received.swap(pending_rgb);w=pending_w;h=pending_h;spp=pending_spp;
            }
        }
        if(!received.empty()){
            if(!texture || w!=tex_w || h!=tex_h){
                if(texture)SDL_DestroyTexture(texture);
                texture=SDL_CreateTexture(display,SDL_PIXELFORMAT_RGB24,SDL_TEXTUREACCESS_STREAMING,w,h);
                if(!texture)throw std::runtime_error(SDL_GetError());
                tex_w=w;tex_h=h;
            }
            SDL_UpdateTexture(texture,nullptr,received.data(),w*3);
            current_rgb=std::move(received);
            rendered_spp=spp;
        }
        const mpt::Vec3 light_center=mpt::Vec3{0,1.99f,-0.39f}+light_offset;
        std::ostringstream lamp_status;
        lamp_status<<std::fixed<<std::setprecision(2)
                   <<light_center.x<<","<<light_center.y<<","<<light_center.z;
        std::string title="MiniPathTracer | "+std::to_string(rendered_spp)+" spp | "
            +std::to_string(tex_w)+"x"+std::to_string(tex_h)
            +" | lamp XYZ ("+lamp_status.str()+") "
            +std::to_string(static_cast<int>(light_scale*100))
            +"% | Arrows/PgUp/PgDn move lamp, Home resets, [ ] brightness, F12 save";
        SDL_SetWindowTitle(window,title.c_str());
        SDL_SetRenderDrawColor(display,15,18,20,255);
        SDL_RenderClear(display);
        if(texture)SDL_RenderCopy(display,texture,nullptr,nullptr);
        SDL_RenderPresent(display);
        SDL_Delay(12);
    }
    cancel.store(true);
    if(job.joinable())job.join();
    if(texture)SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(display);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

#endif
}
int main(int argc,char** argv){
    try{
        Cli opt=parse(argc,argv);
        mpt::Scene scene;scene.build_cornell_box();
        std::cout<<"MiniPathTracer | "<<scene.triangles.size()<<" triangles, "<<scene.bvh_node_count()<<" BVH nodes\n";
        mpt::Renderer renderer(std::move(scene));
#ifdef MPT_WITH_SDL2
        if(!opt.headless)preview(renderer,opt);
        else headless(renderer,opt);
#else
        if(!opt.headless)std::cout<<"SDL2 not found at build time; running headless.\n";
        headless(renderer,opt);
#endif
        return 0;
    }catch(const std::exception& e){std::cerr<<"Error: "<<e.what()<<"\nUse --help for options.\n";return 1;}
}