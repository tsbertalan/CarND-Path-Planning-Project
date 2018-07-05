#ifndef GRAPH1010
#define GRAPH1010

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "utils.h"

using namespace std;
#define GRAPH_ENABLED true

class plot {
public:
    FILE *gp;
    bool enabled, persist;

    plot(bool _persist = false, bool _enabled = GRAPH_ENABLED) {
        enabled = _enabled;
        persist = _persist;
        if (enabled) {
            if (persist)
                gp = popen("gnuplot -persist", "w");
            else
                gp = popen("gnuplot", "w");
        }
    }

    void plot_data(vector<float> x, const char *style = "points", const char *title = "Data") {
        if (!enabled)
            return;
        fprintf(gp, "set title '%s' \n", title);
        fprintf(gp, "plot '-' w %s \n", style);
        for (int k = 0; k < x.size(); k++) {
            fprintf(gp, "%f\n", x[k]);
        }
        fprintf(gp, "e\n");
        fflush(gp);
    }

    void plot_data(vector<float> x, vector<float> y, const char *style = "points", const char *title = "Data") {
        if (!enabled)
            return;
        fprintf(gp, "set title '%s' \n", title);
        fprintf(gp, "plot '-' w %s \n", style);
        for (int k = 0; k < x.size(); k++) {
            fprintf(gp, "%f %f \n", x[k], y[k]);
        }
        fprintf(gp, "e\n");
        fflush(gp);
    }

    void plot_data(vector<vector<float>> X, vector<vector<float>> Y, vector<string> styles = {"points"},
                   const char *title = "Data", bool fixed_y = true) {
        if (!enabled)
            return;

        fprintf(gp, "set title '%s' \n", title);
        const vector<string> colors = {"black", "blue", "red", "green", "magenta", "cyan", "yellow"};

        // Use a smoothly sliding box.
        vector<float> lows_x, lows_y, highs_x, highs_y;
        for (int i = 0; i < X.size(); i++) {
            auto x = X[i];
            auto y = Y[i];
            lows_x.push_back(min(x));
            lows_y.push_back(min(y));
            highs_x.push_back(max(x));
            highs_y.push_back(max(y));
        }
        fprintf(gp, "set xrange [%f:%f] \n", min(lows_x), max(highs_x));
        if (fixed_y) {
//            float extreme = max(fabs(min(lows_y)), fabs(max(highs_y)));
            float extreme = 12;
            fprintf(gp, "set yrange [%f:%f] \n", -extreme, extreme);//low_y + box_width_y);
        } else {
            fprintf(gp, "set yrange [%f:%f] \n", min(lows_y), max(highs_y));
        }


        fprintf(gp, "set palette rgb 23,28,3 \n");
        fprintf(gp, "set key off \n");

        fprintf(gp, "plot");
        for (int i = 0; i < X.size(); i++) {
            string color = colors[i % colors.size()];
            string style = styles[i % styles.size()];
            fprintf(gp, " '-' with %s linecolor palette linewidth 2,", style.c_str());
        }
        fprintf(gp, "\n");

        for (int i = 0; i < X.size(); i++) {
            vector<float> x = X[i];
            vector<float> y = Y[i];
            for (int k = 0; k < y.size(); k++) {
                fprintf(gp, "%f %f %d \n", x[k], y[k], k);
            }
            fprintf(gp, "e\n");
        }
        fflush(gp);
    }

    ~plot() {
        if (enabled)
            pclose(gp);
    }

};

/*
int main(int argc,char **argv) {
 plot p;
 for(int a=0;a<100;a++) {
 vector<float> x,y;
 for(int k=a;k<a+200;k++) {
   x.push_back(k);
   y.push_back(k*k);
 }
  p.plot_data(x,y);
 }
 return 0;
}
*/

#endif
