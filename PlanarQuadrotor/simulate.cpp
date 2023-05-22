/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include <matplot/matplot.h>
#include "simulate.h"
using namespace matplot;

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 10, 10, 10, 1, 10, 0.25 / 2 / M_PI;
    R.row(0) << 0.1, 0.05;
    R.row(1) << 0.05, 0.1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * DONE 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * DONE 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0; // pocztek
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);
    /**
     * TODO: Plot x, y, theta over time
     * DONE 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * DONE 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;
    int history_save = 0;
    Eigen::VectorXf help_save = Eigen::VectorXf::Zero(6);
    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float x1, y1;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
        
        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                // else if (e.type == SDL_MOUSEMOTION)
                // {
                //     SDL_GetMouseState(&x, &y);
                //     std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                // }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    if (e.button.button == SDL_BUTTON_LEFT)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                        x1 = x;
                        y1 = y;
                        x1 = (x1-640)/1280;
                        y1 = (y1-360)/760;
                        state << x1, y1, 0, 0, 0, 0;                        
                        quadrotor.SetGoal(state);
                        
                    }
                }
                
            }
            /* creating a history save*/
            
            if(history_save % 10 == 0)
            {
                help_save = quadrotor.GetState();
                x_history.push_back(help_save[0]);
                y_history.push_back(help_save[1]);
                time_history.push_back(history_save/5000);
                theta_history.push_back(help_save[2]);
            }
            history_save++;

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */ // WYSWIETLNIE (PREZENTACJA) QUADRATORA
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            
        }
    }
    SDL_Quit();

    auto graph_x = plot(x_history,time_history);
    title("x history");
    show();

    auto graph_y = plot(y_history,time_history);
    title("y history");
    show();

    auto graph_theta = plot(theta_history,time_history);
    title("theta history");
    show();

    char choice;
    std::cout << "You can see a graph of trajectory, click 'p'" << std::endl;
    while(std::cin >> choice)
    {
       if(choice == 'p')
       {
        auto graph_trajectory = plot3(x_history,y_history,time_history);
        title("trajectory");
        show();
       } 
       else if(choice == 'e') break;
       std::cout << "If you want to quit, you click 'e'." << std::endl;
    }

    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}