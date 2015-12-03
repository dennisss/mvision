#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <vector>

using namespace std;

char *spacetok(char *str);

istream& operator>>(istream& is, vector<string>& toks);

bool exists(string filename);

#endif
