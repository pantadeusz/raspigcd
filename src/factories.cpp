#include "factories.hpp"

#include <converters/gcd_program_to_steps.hpp>
#include <gcd/remove_g92_from_gcode.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/motor_layout.hpp>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::hardware;
using namespace raspigcd::gcd;

/// Visualization part
#ifdef HAVE_SDL2
#include <SDL2/SDL.h>
#include <video.hpp>
class video_sdl
{
public:
    std::shared_ptr<SDL_Window> window;
    std::shared_ptr<SDL_Renderer> renderer;

    std::atomic<bool> active;
    std::atomic<double> spindle_power;

    std::thread loop_thread;

    distance_t current_position;

    std::list<distance_t> movements_track;
    steps_t steps_scale;

    std::mutex list_mutex;
    configuration::global* cfg;
    std::shared_ptr<motor_layout> ml;
    driver::low_buttons_fake* buttons_drv;

    int z_p_x; // 1000x
    int z_p_y; // 1000x
    int view_x;
    int view_y;
    int scale_view;

    steps_t position_for_fake;

    bool dragging_view = false;
    bool scaling_view = false;

    std::vector<int> previous_button_state;

    void set_steps(const steps_t& st)
    {
        if (!(position_for_fake == st)) {
            current_position = ml->steps_to_cartesian(st);
            steps_t reduced_a;
            steps_t reduced_b;

            std::lock_guard<std::mutex> guard(list_mutex);
            for (std::size_t i = 0; i < current_position.size(); i++) {
                reduced_a[i] = current_position[i] * 10.0;       ///(int)(cfg->steppers[i].steps_per_mm);
                reduced_b[i] = movements_track.back()[i] * 10.0; ///(int)(cfg->steppers[i].steps_per_mm);
            }
            if (!(reduced_a == reduced_b)) {
                movements_track.push_back(current_position);
            }

            if (current_position[0] < -10) { // 10mm left
                if (previous_button_state[0] != 1) buttons_drv->trigger_button_down(0);
                previous_button_state[0] = 1;
            } else {
                if (previous_button_state[0] != 0) buttons_drv->trigger_button_up(0);
                previous_button_state[0] = 0;
            }
            if (current_position[1] > 10) { // 10mm forward (y positive)
                if (previous_button_state[1] != 1) buttons_drv->trigger_button_down(1);
                previous_button_state[1] = 1;
            } else {
                if (previous_button_state[1] != 0) buttons_drv->trigger_button_up(1);
                previous_button_state[1] = 0;
            }
            if (current_position[2] > 90) { // 90mm up
                if (previous_button_state[2] != 1) buttons_drv->trigger_button_down(2);
                previous_button_state[2] = 1;
            } else {
                if (previous_button_state[2] != 0) buttons_drv->trigger_button_up(2);
                previous_button_state[2] = 0;
            }
        }

        position_for_fake = st;
    }

    void draw_path(std::shared_ptr<SDL_Renderer> renderer, int width, int /*height*/, const std::list<distance_t>& t)
    {
        static std::random_device dev;
        static std::mt19937 rng(dev());
        static std::uniform_int_distribution<std::mt19937::result_type> draw_upper(0, 100); // distribution in range [1, 6]


        std::map<int, int> z_buffer;

        for (auto& e : t) {
            double x = e[0];
            double y = e[1];
            double z = e[2];
            if ((e[2] <= 0) || (draw_upper(rng) == 0)) {
                //                if ((z_buffer.count(y * width + x) == 0) || (z_buffer[y * width + x] >= z)) {
                if (true) { //(z_buffer.count(y * width + x) == 0) || (z_buffer[y * width + x] >= z)) {
                    SDL_SetRenderDrawColor(renderer.get(), 255 - (e[2] * 255 / 5), 255, 255, 255);
                    SDL_RenderDrawPoint(renderer.get(), (x * 1000 / scale_view + view_x) + z * z_p_x / scale_view, (-y * 1000 / scale_view + view_y) - z * z_p_y / scale_view);
                    z_buffer[y * width + x] = z;
                }
            }
        }
    }

    video_sdl(configuration::global* cfg_, driver::low_buttons_fake* buttons_drv_, int width = 640, int height = 480)
    {
        if (SDL_Init(SDL_INIT_VIDEO) != 0) throw std::invalid_argument("SDL_Init");
        previous_button_state.resize(100);
        buttons_drv = buttons_drv_;

        scale_view = 1000;
        active = true;
        cfg = cfg_;

        z_p_x = 500; // 1000x
        z_p_y = 500; // 1000x
        view_x = width / 2;
        view_y = height / 2;

        ml = motor_layout::get_instance(*cfg);
        movements_track.push_back(distance_t());
        loop_thread = std::thread([this, width, height]() {
            std::cout << "loop thread..." << std::endl;

            window = std::shared_ptr<SDL_Window>(SDL_CreateWindow("GCD Execution Simulator By Tadeusz Puzniakowski",
                                                     SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                                     width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE),
                [](SDL_Window* ptr) {
                    SDL_DestroyWindow(ptr);
                });
            if (window == nullptr) throw std::invalid_argument("SDL_CreateWindow - error");

            renderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED), [](SDL_Renderer* ptr) {
                SDL_DestroyRenderer(ptr);
            });
            if (renderer == nullptr) throw std::invalid_argument("SDL_CreateRenderer");

            for (; active;) {
                SDL_Event event;
                while (SDL_PollEvent(&event)) {
                    int k;
                    switch (event.type) {
                    case SDL_QUIT:
                        active = false;
                        std::cout << "window closed" << std::endl;
                        buttons_drv->trigger_button_down(low_buttons_default_meaning_t::TERMINATE); // stop execution

                        break;
                    case SDL_KEYDOWN:
                        k = event.key.keysym.sym - SDLK_0;
                        if ((k >= 0) && (k < 10)) {
                            buttons_drv->trigger_button_down(k);
                        } else if (event.key.keysym.sym == SDLK_LEFT) {
                            view_x += 10;
                        } else if (event.key.keysym.sym == SDLK_RIGHT) {
                            view_x -= 10;
                        } else if (event.key.keysym.sym == SDLK_UP) {
                            view_y += 10;
                        } else if (event.key.keysym.sym == SDLK_DOWN) {
                            view_y -= 10;
                        } else if (event.key.keysym.sym == SDLK_a) {
                            scale_view += 10;
                        } else if (event.key.keysym.sym == SDLK_q) {
                            scale_view -= 10;
                        }
                        break;
                    case SDL_MOUSEBUTTONDOWN:
                        if (event.button.button == SDL_BUTTON_LEFT) dragging_view = true;
                        if (event.button.button == SDL_BUTTON_MIDDLE) scaling_view = true;
                        break;
                    case SDL_MOUSEBUTTONUP:
                        if (event.button.button == SDL_BUTTON_LEFT) dragging_view = false;
                        if (event.button.button == SDL_BUTTON_MIDDLE) scaling_view = false;
                        break;
                    case SDL_MOUSEMOTION:
                        if (dragging_view) {
                            view_x += event.motion.xrel;
                            view_y += event.motion.yrel;
                        }
                        if (scaling_view) {
                            scale_view += event.motion.yrel;
                        }

                        break;
                    case SDL_KEYUP:
                        k = event.key.keysym.sym - SDLK_0;
                        if ((k >= 0) && (k < 10)) buttons_drv->trigger_button_up(k);
                        break;
                    }
                }

                SDL_SetRenderDrawColor(renderer.get(), 0, 0, 0, 255);
                SDL_RenderClear(renderer.get());

                SDL_SetRenderDrawColor(renderer.get(), 255, 255, 255, 255);
                distance_t s;
                std::list<distance_t> t;

                {
                    std::lock_guard<std::mutex> guard(list_mutex);
                    s = current_position;
                    t = movements_track;
                }
                draw_path(renderer, width, height, t);
                SDL_SetRenderDrawColor(renderer.get(), std::min(255.0, spindle_power * 255), 255 - (std::min(255.0, spindle_power * 255)), 0, 255);
                for (double i = 0; i < 1.0; i += 0.05) {
                    SDL_RenderDrawPoint(renderer.get(),
                        s[0] * 1000 / scale_view + view_x + (i * 5.0 + s[2]) * z_p_x / scale_view,
                        -s[1] * 1000 / scale_view + view_y + (-i * 5.0 + -s[2]) * z_p_y / scale_view);
                }

                {
                    SDL_SetRenderDrawColor(renderer.get(), 16, 128, 32, 255);
                    std::stringstream o;
                    o << "" << s[0] << "\n"
                      << s[1] << "\n"
                      << s[2] << "\n view: " << view_x << "," << view_y << " s: " << scale_view;
                    sdl_draw_text(renderer.get(), 5, 5, o.str());
                }
                SDL_RenderPresent(renderer.get());
                SDL_Delay(33);
            }
        });
    }
    void set_spindle(double spindle_power_level_)
    {
        spindle_power = spindle_power_level_;
    }
    virtual ~video_sdl()
    {
        active = false;
        loop_thread.join();
        SDL_Quit();
    }
};
#else
class video_sdl
{
public:
    std::atomic<bool> active;
    void set_spindle(double spindle_power_level_)
    {
    }
    void set_steps(const steps_t& st)
    {
    }

    video_sdl(configuration::global* cfg_, driver::low_buttons_fake* buttons_drv, int width = 640, int height = 480)
    {
        active = true;
    }
    virtual ~video_sdl()
    {
        active = false;
    }
};
#endif
/// end of visualization


namespace raspigcd {

#ifdef HAVE_SDL2
std::shared_ptr<video_sdl> video;
#endif
/*
std::tuple<
    std::shared_ptr<raspigcd::hardware::low_timers>,
    std::shared_ptr<raspigcd::hardware::low_steppers>,
    std::shared_ptr<raspigcd::hardware::low_spindles_pwm>,
    std::shared_ptr<raspigcd::hardware::low_buttons>,
    std::shared_ptr<raspigcd::hardware::motor_layout>,
    std::shared_ptr<raspigcd::hardware::stepping_simple_timer>>
    */
execution_objects_t stepping_simple_timer_factory(configuration::global cfg)
{
#ifdef HAVE_SDL2
    bool enable_video = false;
#endif
    std::shared_ptr<low_steppers> steppers_drv;
    std::shared_ptr<low_spindles_pwm> spindles_drv;
    std::shared_ptr<low_buttons> buttons_drv;
    std::shared_ptr<raspigcd::hardware::low_timers> timer_drv;

    try {
        auto rp = std::make_shared<driver::raspberry_pi_3>(cfg);
        steppers_drv = rp;
        spindles_drv = rp;
        buttons_drv = rp;
    } catch (const std::invalid_argument e) {
        std::cerr << "verry bad runtime error. Please check configuration file: " << e.what() << std::endl;
	throw e;
    } catch (...) {
        std::cerr << "stepping_simple_timer_factory: exception during raspberry_pi_3 object initialization. Fallback to GUI" << std::endl;
        auto fk = std::make_shared<driver::inmem>();
        steppers_drv = fk;
        spindles_drv = std::make_shared<raspigcd::hardware::driver::low_spindles_pwm_fake>(
            [](const int s_i, const double p_i) {
                std::cout << "SPINDLE " << s_i << " POWER: " << p_i << std::endl;
#ifdef HAVE_SDL2
                if (video.get() != nullptr) video->set_spindle(p_i);
#endif
            });
        fk->on_enable_steppers = [](const std::vector<bool> m) {
            std::cout << "STEPPERS: ";
            for (auto e : m) {
                std::cout << (e ? "+" : " ");
            }
            std::cout << ";" << std::endl;
        };
        auto buttons_fake_fake = std::make_shared<driver::low_buttons_fake>(10);
        buttons_drv = std::make_shared<driver::low_buttons_fake>(10);

#ifdef HAVE_SDL2
        fk->set_step_callback([](const steps_t& st) {
            if (video.get() != nullptr) video->set_steps(st);
        });
        enable_video = true;
#endif
    }
    //std::shared_ptr<driver::raspberry_pi_3> steppers_drv = std::make_shared<driver::raspberry_pi_3>(cfg);
    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    motor_layout_->set_configuration(cfg);

    switch (cfg.lowleveltimer) {
    case raspigcd::configuration::low_timers_e::WAIT_FOR:
        timer_drv = std::make_shared<hardware::driver::low_timers_busy_wait>();
        break;
    case raspigcd::configuration::low_timers_e::BUSY_WAIT:
        timer_drv = std::make_shared<hardware::driver::low_timers_wait_for>();
        break;
    case raspigcd::configuration::low_timers_e::FAKE:
        timer_drv = std::make_shared<hardware::driver::low_timers_fake>();
        break;
    }
    std::shared_ptr<stepping_simple_timer> stepping = std::make_shared<stepping_simple_timer>(cfg, steppers_drv, timer_drv);
#ifdef HAVE_SDL2
    if (enable_video)
        video = std::make_shared<video_sdl>(&cfg, (driver::low_buttons_fake*)buttons_drv.get());
#endif
    return {
        timer_drv,
        steppers_drv,
        spindles_drv,
        buttons_drv,
        motor_layout_,
        stepping
    };
}
} // namespace raspigcd
