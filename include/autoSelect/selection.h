#pragma once

#include <string>

//selector configuration
#define HUE 36
#define DEFAULT 1
#define AUTONS "L", "R", "LM", "RM", "SL", "SR"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
