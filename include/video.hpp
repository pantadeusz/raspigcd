#ifndef __VIDEO_HELPER_HPP_____
#define __VIDEO_HELPER_HPP_____

//#define HAVE_SDL2

#ifdef HAVE_SDL2

#include <string>
#include <SDL2/SDL.h>

extern const unsigned char simple_font[2048];



auto  sdl_draw_char = [](SDL_Renderer *renderer, const int x0, const int y0, const char c,const  unsigned char * font) -> void {
//8
for (int y = 0; y < 8; y++) {
for (int x = 0; x < 8; x++) {
    //SDL_SetRenderDrawColor(renderer, , 255, 255, 255);
    if (font[y+c*8] & (1<<(8-x))) SDL_RenderDrawPoint(renderer, x+x0, y+y0);
}
}
};


inline void sdl_draw_text(SDL_Renderer *renderer, int x0, int y0, std::string txt) {
//8
int x = x0;
int y = y0;
for (auto c : txt) {
    if (c == '\n') {
        y+=8;
        x=x0;
    }else{
        sdl_draw_char(renderer,x,y,c, simple_font);
        x+=8;
    }
    
}
}

#endif

#endif
