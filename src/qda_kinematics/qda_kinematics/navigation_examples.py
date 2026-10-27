"""Named, reachable inspection targets for the expanded factory."""
import math

NAVIGATION_TARGETS={
    'tour_west':(-.70,.80,math.pi),
    'tour_sw':(-.70,-1.95,-math.pi/2),
    'tour_south_mid':(1.2,-1.95,0.),
    'tour_south':(2.5,-1.95,0.),
    'tour_east_mid':(4.2,-1.95,0.),
    'tour_se':(5.95,-1.95,math.pi/2),
    'tour_ne':(5.95,2.15,math.pi/2),
    'tour_north':(3.5,2.15,math.pi),
    'tour_nw':(-.70,2.15,math.pi),
    'survey_west_north':(-.70,.80,math.pi/2),
    'survey_west_east':(-.70,.80,0.),
    'survey_east_south':(3.7,1.8,-math.pi/2),
    'survey_east_east':(3.7,1.8,0.),
    'survey_east_north':(3.7,1.8,math.pi/2),
}
PERIMETER_TOUR=['tour_south_mid','tour_south','tour_east_mid','tour_se','tour_ne','tour_north','tour_nw','tour_west','tour_south_mid']
MULTI_GOAL_TOUR=['inspection','survey_west_north','survey_west_east','north_entry',
                 'survey_east_north','survey_east_east','survey_east_south','south_entry','start']
