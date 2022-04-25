#pragma once

#include <string>

//selector configuration
#define HUE 360
#define DEFAULT 4
#define AUTONS "L", "R", "LM", "RM", "SL", "SR"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
