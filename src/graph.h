#ifndef GRAPH1010
#define GRAPH1010

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "utils.h"

#include <sstream>

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

  void plot_data(vector<double> x, const char *style = "points", const char *title = "Data") {
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

  void plot_data(vector<double> x, vector<double> y, const char *style = "points", const char *title = "Data") {
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

  void plot_data(vector<vector<double>> X, vector<vector<double>> Y,
                 const char *xlabel, const char *ylabel,
                 vector<string> styles = {"points"},
                 const char *title = "Data", const char *cblabel = "", vector<vector<double>> C = {{}},
                 vector<double> ylims = {12, 0}, vector<double> xlims = {0, 72}
  ) {
    if (!enabled)
      return;

    fprintf(gp, "set title '%s' \n", title);
    const vector<string> colors = {"black", "blue", "red", "green", "magenta", "cyan", "yellow"};

    // Use a smoothly sliding box.
    vector<double> lows_x, lows_y, highs_x, highs_y;
    for (int i = 0; i < X.size(); i++) {
      auto x = X[i];
      auto y = Y[i];
      lows_x.push_back(min(x));
      lows_y.push_back(min(y));
      highs_x.push_back(max(x));
      highs_y.push_back(max(y));
    }
    if (xlims.size()==2)
      fprintf(gp, "set xrange [%f:%f] \n", xlims[0], xlims[1]);
    else
      fprintf(gp, "set xrange [%f:%f] \n", min(lows_x), max(highs_x));
    if (ylims.size()==2)
      fprintf(gp, "set yrange [%f:%f] \n", ylims[0], ylims[1]);
    else
      fprintf(gp, "set yrange [%f:%f] \n", min(lows_y), max(highs_y));

    // Try to make the figure size roughly proprotional to the limits.
    if (xlims.size()==2 && ylims.size()==2) {
      double dx = abs(xlims[1] - xlims[0]);
      double dy = abs(ylims[1] - ylims[0]);
      cout << "dx=" << dx << ", dy=" << dy << endl;
      double width, height;
      height = 300;
      width = min(1900., dx/dy*height);
      ostringstream oss;
      oss << "set terminal x11 size " << (int) width << "," << (int) height << " \n";
      fprintf(gp, "%s", oss.str().c_str());
    }

    fprintf(gp, "set palette rgb 33,13,10 \n");
    fprintf(gp, "set key off \n");
    fprintf(gp, "set xlabel '%s' \n", xlabel);
    fprintf(gp, "set ylabel '%s' \n", ylabel);
    if (strlen(cblabel) > 0)
      fprintf(gp, "set cblabel '%s' \n", cblabel);

    fprintf(gp, "plot");
    for (int i = 0; i < X.size(); i++) {
      string color = colors[i%colors.size()];
      string style = styles[i%styles.size()];
      fprintf(gp, " '-' with %s linecolor palette linewidth 1,", style.c_str());
    }
    fprintf(gp, "\n");

    for (int i = 0; i < X.size(); i++) {
      vector<double> x = X[i];
      vector<double> y = Y[i];
      vector<double> c = C[i%C.size()];
      for (int k = 0; k < y.size(); k++) {
        double cv = c[min(k, (int) c.size() - 1)];
        fprintf(gp, "%f %f %f \n", x[k], y[k], cv);
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
 vector<double> x,y;
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
