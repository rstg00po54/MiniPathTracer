#include "pathtracer.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
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
      <<"  --out FILE      output image (BMP or PPM)\n"
      <<"  --headless      no SDL2 preview (always works)\n"
      <<"  --help          show this text\n\n"
      <<"SDL2 preview: +/- exposure; S save; Esc close.\n";
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
        else if(a=="--out")opt.out=take(i);
        else if(a=="--headless")opt.headless=true;
        else if(a=="--help"||a=="-h"){help();std::exit(0);}
        else throw std::invalid_argument("Unknown option: "+a);
    }
    auto& s=opt.settings;
    if(s.width<1||s.height<1||s.width>4096||s.height>4096||s.samples<1||s.samples>100000||s.max_bounces<1||s.max_bounces>64||s.threads<0||s.threads>256||s.exposure<=0)
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
    SDL_Window* window=SDL_CreateWindow("MiniPathTracer | Cornell Box", SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
       std::max(480,opt.settings.width),std::max(480,opt.settings.height),SDL_WINDOW_RESIZABLE);
    if(!window){SDL_Quit();throw std::runtime_error(SDL_GetError());}
    SDL_Renderer* display=SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    if(!display) display=SDL_CreateRenderer(window,-1,SDL_RENDERER_SOFTWARE);
    if(!display){SDL_DestroyWindow(window);SDL_Quit();throw std::runtime_error(SDL_GetError());}
    SDL_Texture* texture=SDL_CreateTexture(display,SDL_PIXELFORMAT_RGB24,SDL_TEXTUREACCESS_STREAMING,
                                           opt.settings.width,opt.settings.height);
    if(!texture){SDL_DestroyRenderer(display);SDL_DestroyWindow(window);SDL_Quit();throw std::runtime_error(SDL_GetError());}
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY,"1");
    mpt::Image image(opt.settings.width,opt.settings.height);
    std::atomic<bool> stop{false},done{false};
    std::mutex mutex;
    std::vector<uint8_t> latest;
    int latest_samples=0;
    std::atomic<float> exposure{opt.settings.exposure};
    auto start=std::chrono::steady_clock::now();
    std::thread worker([&]{
        renderer.render(opt.settings,image,[&](int sample,const mpt::Image& image_ref){
            auto pixels=image_ref.rgb8(exposure.load());
            {std::lock_guard<std::mutex> guard(mutex);latest=std::move(pixels);latest_samples=sample;}
            log_pass(sample,opt.settings,start);
        },&stop);
        done=true;
    });
    bool running=true,dirty=false;
    while(running){
        SDL_Event event;
        while(SDL_PollEvent(&event)){
            if(event.type==SDL_QUIT)running=false;
            if(event.type==SDL_KEYDOWN){
                if(event.key.keysym.sym==SDLK_ESCAPE)running=false;
                if(event.key.keysym.sym==SDLK_s && done.load()){
                    if(image.save(opt.out,exposure.load()))std::cout<<"Saved "<<opt.out<<"\n";
                }
                if(event.key.keysym.sym==SDLK_EQUALS||event.key.keysym.sym==SDLK_PLUS||event.key.keysym.sym==SDLK_KP_PLUS){
                    exposure.store(exposure.load()*1.2f);dirty=true;
                }
                if(event.key.keysym.sym==SDLK_MINUS||event.key.keysym.sym==SDLK_KP_MINUS){
                    exposure.store(exposure.load()/1.2f);dirty=true;
                }
            }
        }
        std::vector<uint8_t> frame;
        int spp=0;
        {std::lock_guard<std::mutex> guard(mutex);
            if(latest_samples>0){frame.swap(latest);spp=latest_samples;latest_samples=0;}
        }
        if(!frame.empty()){
            SDL_UpdateTexture(texture,nullptr,frame.data(),opt.settings.width*3);
            SDL_SetWindowTitle(window,("MiniPathTracer | Cornell Box | "+std::to_string(spp)+" / "+std::to_string(opt.settings.samples)+" spp").c_str());
        }
        // When exposure changes, rerender from the final completed buffer.
        if(dirty&&done.load()){
            frame=image.rgb8(exposure.load());
            SDL_UpdateTexture(texture,nullptr,frame.data(),opt.settings.width*3);
            dirty=false;
        }
        SDL_SetRenderDrawColor(display,15,18,20,255);
        SDL_RenderClear(display);
        SDL_RenderCopy(display,texture,nullptr,nullptr);
        SDL_RenderPresent(display);
        SDL_Delay(10);
    }
    stop=true;
    worker.join();
    if(done.load()&&!image.save(opt.out,exposure.load()))std::cerr<<"Warning: failed to save "<<opt.out<<"\n";
    else if(done.load())std::cout<<"Saved "<<opt.out<<"\n";
    SDL_DestroyTexture(texture);SDL_DestroyRenderer(display);SDL_DestroyWindow(window);SDL_Quit();
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