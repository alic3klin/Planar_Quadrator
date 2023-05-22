#include "planar_quadrotor_visualizer.h"
#include "cmath"

float q_x, q_y, q_theta;
/* dimensions */
const int HEIGHT_planarquadrator = 10;
const int HALF_HEIGHT = HEIGHT_planarquadrator/2;
const int WIDTH_planarquadrator = 160;
const int HALF_WIDTH =  WIDTH_planarquadrator/2;
const int HEIGHT_support_propellers = 25;
const int DISTANCE_support_propellers = 60 ;
const int RADIUS_propellers = 7;
const int RADIUS_propellers2 = 3;
const int CENTRE_circle = 20;
/* color */
const int color_propellers = 0xFFA500FF;   
const int color_propellers2 = 0xFFCCCCFF;  // color of animated propellers

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}
/*
 * TODO: Improve visualizetion
 * DONE 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * DONE 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * DONE 3. Animate propellers (extra points) 
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    /* x, y, theta coordinates */
    state[0] = state[0] * 1280;
    state[1] = state[1] * 760;
    /* coordinate transformation */ 
    q_x = state[0] + 640;
    q_y = state[1] + 360;
    q_theta = state[2];

    // support propellers
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), (q_x + DISTANCE_support_propellers*cos(q_theta)), (q_y - DISTANCE_support_propellers*sin(q_theta)), (q_x - HEIGHT_support_propellers*sin(q_theta) + DISTANCE_support_propellers*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta) - DISTANCE_support_propellers*sin(q_theta)) );
    SDL_RenderDrawLine(gRenderer.get(), (q_x - DISTANCE_support_propellers*cos(q_theta)), (q_y + DISTANCE_support_propellers*sin(q_theta)), (q_x - HEIGHT_support_propellers*sin(q_theta) - DISTANCE_support_propellers*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta) + DISTANCE_support_propellers*sin(q_theta)) );
    
    // circle - element of propellers
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) + HALF_WIDTH*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta) - HALF_WIDTH*sin(q_theta)), RADIUS_propellers, color_propellers);  // right
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) - HALF_WIDTH*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta) + HALF_WIDTH*sin(q_theta)), RADIUS_propellers, color_propellers);  // left
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta)  + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*sin(q_theta) ), RADIUS_propellers, color_propellers);  // right - from centre
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta)), (q_y - HEIGHT_support_propellers*cos(q_theta)  + (DISTANCE_support_propellers - CENTRE_circle)*sin(q_theta)), RADIUS_propellers, color_propellers);  // left - from centre
    
    // polygon connected circles [right]
    const Sint16 vx_prop1[] = {static_cast<Sint16>(q_x - (HEIGHT_support_propellers + RADIUS_propellers)*sin(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta)), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + HALF_WIDTH*cos(q_theta)), static_cast<Sint16>(q_x - (HEIGHT_support_propellers + RADIUS_propellers)*sin(q_theta) + HALF_WIDTH*cos(q_theta)), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta)) };
    const Sint16 vy_prop1[] = {static_cast<Sint16>(q_y - (RADIUS_propellers + HEIGHT_support_propellers)*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers)*cos(q_theta) - HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers)*cos(q_theta) - HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers)*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*sin(q_theta)) };
    filledPolygonColor(gRenderer.get(), vx_prop1, vy_prop1, 4, color_propellers );
    // polygon connected circles [left]
    const Sint16 vx_prop2[] = {static_cast<Sint16>((q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) - HALF_WIDTH*cos(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta)), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta)), static_cast<Sint16>(q_x - (HEIGHT_support_propellers + RADIUS_propellers)*sin(q_theta) - HALF_WIDTH*cos(q_theta)) };
    const Sint16 vy_prop2[] = {static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers)*cos(q_theta) + HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers)*cos(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers)*cos(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers)*cos(q_theta) + HALF_WIDTH*sin(q_theta)) };
    filledPolygonColor(gRenderer.get(), vx_prop2, vy_prop2, 4, color_propellers );

    // base - deck
    const Sint16 vx_deck[] = {static_cast<Sint16>(q_x - HALF_HEIGHT*sin(q_theta) - HALF_WIDTH*cos(q_theta)), static_cast<Sint16>(q_x - HALF_HEIGHT*sin(q_theta) + HALF_WIDTH*cos(q_theta)), static_cast<Sint16>(q_x + HALF_HEIGHT*sin(q_theta) + HALF_WIDTH*cos(q_theta)), static_cast<Sint16>(q_x + HALF_HEIGHT*sin(q_theta) - HALF_WIDTH*cos(q_theta)) };
    const Sint16 vy_deck[] = {static_cast<Sint16>(q_y - HALF_HEIGHT*cos(q_theta) + HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y - HALF_HEIGHT*cos(q_theta) - HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y + HALF_HEIGHT*cos(q_theta) - HALF_WIDTH*sin(q_theta)), static_cast<Sint16>(q_y + HALF_HEIGHT*cos(q_theta) + HALF_WIDTH*sin(q_theta)) };
    filledPolygonRGBA(gRenderer.get(),vx_deck, vy_deck, 4, 0x00, 0x00, 0x00, 0xFF);

    /* A N I M A T I O N   -   P R O P P E R E L E R S */

    // ANIMATION circle - element of propellers
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) + HALF_WIDTH*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), (q_y - HEIGHT_support_propellers*cos(q_theta) + (RADIUS_propellers - HALF_WIDTH)*sin(q_theta)), RADIUS_propellers2, color_propellers2);  // right
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) - HALF_WIDTH*cos(q_theta) + abs(CENTRE_circle*sin(q_theta))), (q_y - HEIGHT_support_propellers*cos(q_theta) + (HALF_WIDTH - RADIUS_propellers)*sin(q_theta)), RADIUS_propellers2, color_propellers2);  // left
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta) + abs(CENTRE_circle*sin(q_theta))), (q_y - HEIGHT_support_propellers*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers - RADIUS_propellers)*sin(q_theta)), RADIUS_propellers2, color_propellers2);  // right - from centre
    filledCircleColor(gRenderer.get(), (q_x - HEIGHT_support_propellers*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), (q_y - HEIGHT_support_propellers*cos(q_theta) + (DISTANCE_support_propellers - CENTRE_circle + RADIUS_propellers)*sin(q_theta)), RADIUS_propellers2, color_propellers2);  // left - from centre
    // ANIMATION - polygon connected circles [right]
    const Sint16 vx_propA1[] = {static_cast<Sint16>(q_x - (RADIUS_propellers + HEIGHT_support_propellers)*sin(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta) + abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + HALF_WIDTH*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers + RADIUS_propellers)*sin(q_theta) + HALF_WIDTH*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (DISTANCE_support_propellers - CENTRE_circle)*cos(q_theta)  + abs(CENTRE_circle*sin(q_theta))) };
    const Sint16 vy_propA1[] = {static_cast<Sint16>(q_y - (RADIUS_propellers2 + HEIGHT_support_propellers)*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers - RADIUS_propellers)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers2)*cos(q_theta) + (RADIUS_propellers - HALF_WIDTH)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers2)*cos(q_theta) + (RADIUS_propellers - HALF_WIDTH)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers2)*cos(q_theta) + (CENTRE_circle - DISTANCE_support_propellers - RADIUS_propellers)*sin(q_theta)) };
    filledPolygonColor(gRenderer.get(), vx_propA1, vy_propA1, 4, color_propellers2 );
    // ANIMATION - polygon connected circles [left]
    const Sint16 vx_propA2[] = {static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) - HALF_WIDTH*cos(q_theta) + abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers - RADIUS_propellers)*sin(q_theta) + (CENTRE_circle - DISTANCE_support_propellers)*cos(q_theta) - abs(CENTRE_circle*sin(q_theta))), static_cast<Sint16>(q_x - (HEIGHT_support_propellers + RADIUS_propellers)*sin(q_theta) - HALF_WIDTH*cos(q_theta) + abs(CENTRE_circle*sin(q_theta))) };
    const Sint16 vy_propA2[] = {static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers2)*cos(q_theta) + (HALF_WIDTH - RADIUS_propellers)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers2)*cos(q_theta) + (DISTANCE_support_propellers - CENTRE_circle + RADIUS_propellers)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers - RADIUS_propellers2)*cos(q_theta) + (DISTANCE_support_propellers - CENTRE_circle+ RADIUS_propellers)*sin(q_theta)), static_cast<Sint16>(q_y - (HEIGHT_support_propellers + RADIUS_propellers2)*cos(q_theta) + HALF_WIDTH*sin(q_theta)) };
    filledPolygonColor(gRenderer.get(), vx_propA2, vy_propA2, 4, color_propellers2 );

}