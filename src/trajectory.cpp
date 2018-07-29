//
// Created by tsbertalan on 7/4/18.
//

#include "trajectory.h"
#include "parameters.h"

using namespace std;

Trajectory::Trajectory(CoordinateTransformer *transform)
    : transform(transform) {}

//Trajectory::Trajectory(const Trajectory &parent) : transform(parent.transform) {
//    this->segments = parent.segments;
//}

Trajectory
Trajectory::generate_extension(FrenetPose current, double t_reuse, double t_replan, double DT, double DS,
                               double sp, double d, double spp,
                               double dp, double dpp) {

  // Get the initial state for the JMT.
  FullState begin;
  if (segments.size()==0) {
    begin.s.y = current.s;
    begin.s.yp = 0;
    begin.s.ypp = 0;
    begin.d.y = current.d;
    begin.d.yp = 0;
    begin.d.ypp = 0;
    t_replan = 0;
    t_reuse = 0;

  } else {
    begin = state(t_replan);
  }


  // Construct the final state for the JMT.
  FullState end;
  end.s.yp = sp;
  end.s.ypp = spp;
  end.d.y = d;
  end.d.yp = dp;
  end.d.ypp = dpp;

  // If the distance to drive wasn't given, project using the mean speed.
  if (DS==-1) {
    double mean_s_vel = (begin.s.yp + sp)/2;
    DS = mean_s_vel*DT;
  }
  end.s.y = begin.s.y + DS;

  // Copy what we need to the new child trajectory.
  Trajectory child(transform);
  child.segments = segments;

  // Only keep segments that overlap with [t_reuse, t_replan],
  // and trim the final segment's remit to end with t_replan.
  child.cut_start(t_reuse, t_replan);

  // Now, work in the trimmed frame.
  t_replan -= t_reuse;
  t_reuse = 0;

  // Construct the new trajectory segment.
  child.segments.push_back(
      {
          .f=TrajectorySegment(t_replan, DT, begin, end),
          .t_responsible_0=t_replan,
          .t_responsible_1=t_replan + DT
      }
  );

  return child;

}

void Trajectory::cut_start(double t_reuse, double t_replan) {

  // Cut off some of our current path segments.
  vector<SegmentRemit> new_segments;
  for (SegmentRemit remit : segments) {

    // Shift the segments.
    remit.f.t_offset += t_reuse;
    remit.t_responsible_0 -= t_reuse;
    remit.t_responsible_1 -= t_reuse;

    // For this remit/loop, all times are now in the new, shifted frame.
    double t_replan_shifted = t_replan - t_reuse;
    double t_reuse_shifted = 0;

    // Considering segments that extend into the reuse time ...
    if (remit.t_responsible_1 > t_reuse_shifted) {

      // If the segment extends into our replanning domain,
      // reduce its responsibility.
      remit.t_responsible_0 = max(0.0, min(remit.t_responsible_0, t_replan_shifted));
      remit.t_responsible_1 = min(remit.t_responsible_1, t_replan_shifted);

      // If the remit is now ~zero-length (or less?) trash it.
      // Use an approximately-zero but finite-positive threshold
      // to account for possible floating-point inaccuracy (.2600000000000001 ~= .259999999999999)
      if (remit.t_responsible_1 - remit.t_responsible_0 < 1e-6)
        continue;

      // Finally, if the segment still overlaps with our un-replanned domain, keep it.
      if (remit.t_responsible_1 > t_reuse_shifted && remit.t_responsible_0 < t_replan_shifted)
        new_segments.push_back(move(remit));
    }
  }

  // Save the new, trimmed segments.
  segments = move(new_segments);
}

double Trajectory::t_max() {
  double out = 0;
  if (segments.size() > 0) {
    out = segments.back().t_responsible_1;
  }
  return out;
}

WorldPose Trajectory::world(double t) {
  return transform->to_world(frenet(t));
}

FrenetPose Trajectory::frenet(double t) {
  double yaw = atan2(dp(t), sp(t));
  return {.s=s(t), .d=d(t), .yaw=yaw};
}

SegmentRemit &Trajectory::get_remit(double t) {

  for (SegmentRemit &candidate : segments) {
    if (t >= candidate.t_responsible_0 && t < candidate.t_responsible_1)
      return candidate;
  }

  return segments[segments.size() - 1];
}

FullState Trajectory::state(double t) {
  return {
      .s={.y=s(t), .yp=sp(t), .ypp=spp(t)},
      .d={.y=d(t), .yp=dp(t), .ypp=dpp(t)}
  };
}

double Trajectory::s(double t, bool ignore_tmax) {
  if (DO_CACHE) {
    auto search = s_cache.find(t);
    if (search!=s_cache.end())
      return search->second;
  }

  TrajectorySegment segment = get_remit(t).f;
  double s_out;
  if (ignore_tmax || t < t_max()) {
    s_out = segment(t).s;
  } else {
    s_out = s(t_max(), true) + sp(t_max(), true)*(t - t_max());
  }
  s_out = fmod(s_out, transform->max_s);

  if (DO_CACHE) s_cache.emplace(t, s_out);

  return s_out;
}

double Trajectory::d(double t, bool ignore_tmax) {
  if (DO_CACHE) {
    auto search = d_cache.find(t);
    if (search!=d_cache.end())
      return search->second;
  }

  TrajectorySegment &segment = get_remit(t).f;
  double d_out;
  if (ignore_tmax || t < t_max()) {
    d_out = segment(t).d;
  } else {
    d_out = d(t_max(), true) + dp(t_max(), true)*(t - t_max());
  }

  if (DO_CACHE) d_cache.emplace(t, d_out);

  return d_out;
}

double Trajectory::sp(double t, bool ignore_tmax) {
//    auto search = sp_cache.find(t);
//    if(search != sp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double t_eval;
  if (ignore_tmax || t < t_max())
    t_eval = t;
  else
    t_eval = min(t, t_max());
  double out = segment.s_derivative(t_eval);

//    sp_cache.emplace(t, out);

  return out;
}

double Trajectory::dp(double t, bool ignore_tmax) {
//    auto search = dp_cache.find(t);
//    if(search != dp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double t_eval;
  if (ignore_tmax || t < t_max())
    t_eval = t;
  else
    t_eval = min(t, t_max());
  double out = segment.d_derivative(t_eval);

//    dp_cache.emplace(t, out);

  return out;
}

double Trajectory::spp(double t) {
//    auto search = spp_cache.find(t);
//    if(search != spp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double out = segment.s_derivative(min(t, t_max()), 2);

//    spp_cache.emplace(t, out);
  return out;
}

double Trajectory::dpp(double t) {
//    auto search = dpp_cache.find(t);
//    if(search != dpp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double out = segment.d_derivative(min(t, t_max()), 2);

//    dpp_cache.emplace(t, out);
  return out;
}

double Trajectory::sppp(double t) {
//    auto search = sppp_cache.find(t);
//    if(search != sppp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double out = segment.s_derivative(min(t, t_max()), 3);

//    sppp_cache.emplace(t, out);
  return out;
}

double Trajectory::dppp(double t) {
//    auto search = dppp_cache.find(t);
//    if(search != dppp_cache.end())
//        return search->second;

  TrajectorySegment segment = get_remit(t).f;
  double out = segment.d_derivative(min(t, t_max()), 3);

//    dppp_cache.emplace(t, out);
  return out;
}

double Trajectory::speed(double t) {

  return distance(0, 0, sp(t), dp(t));
}

double Trajectory::accel(double t) {
  return distance(0, 0, spp(t), dpp(t));
}

double Trajectory::jerk(double t) {
  return distance(0, 0, sppp(t), dppp(t));
}

string Trajectory::dumps() {
  ostringstream oss;
  oss.precision(10);
  oss << "dict(" << endl;

  oss << "segments = [" << endl;

  for (auto segment : segments) {
    oss << "dict(" << endl;
    oss << "t_responsible_0 = " << segment.t_responsible_0 << "," << endl;
    oss << "t_responsible_1 = " << segment.t_responsible_1 << "," << endl;
    oss << "t_pin_0 = " << -segment.f.t_offset << "," << endl;
    oss << "t_pin_1 = " << segment.f.DT - segment.f.t_offset << "," << endl;
    oss << "sdy_xyy_t = np.array([" << endl;
    for (double t = segment.t_responsible_0; t < segment.t_responsible_1; t += .02) {
      FrenetPose fp = segment.f(t);
      WorldPose wp = transform->to_world(fp);
      oss << "[";
      oss << fp.s << "," << fp.d << "," << fp.yaw << ",";
      oss << wp.x << "," << wp.y << "," << wp.yaw << ",";
      oss << t;
      oss << "],";
    }
    oss << "])," << endl;
    oss << ")," << endl;
  }

  oss << "]," << endl;

  oss << ")" << endl;
  return oss.str();
}

void Trajectory::plot(double t_reuse, double t_replan) {
  rand();
  std::ostringstream program_filename;
  program_filename << "trajectory_plot" << rand() << ".py";
  std::ofstream program(program_filename.str());
  program << "import numpy as np, matplotlib.pyplot as plt" << endl;
  program << "data = " << dumps();
  program << "fig, ax = plt.subplots()\n"
             "colors = ['black', 'red', 'cyan', 'magenta', 'green', 'blue', 'gray']\n"
             "xall=[]; yall=[]; tall=[];\n"
             "for iseg, segment in enumerate(data['segments']):\n"
             "    color = colors[iseg % len(colors)]\n"
             "    s, d, yawf, x, y, yaww, t = segment['sdy_xyy_t'].T\n"
             "    xall.extend(x); yall.extend(y); tall.extend(t);\n"
             "    ax.plot(x,  y, color=color, linestyle='-', label=r'$t\\in[%.3f,\\, %.3f)$' % (segment['t_responsible_0'], segment['t_responsible_1']))\n"
             "    ax.scatter(x[0], y[0], edgecolor=color, facecolor=color, s=24, label='$t=%.3f$' % t[0])\n"
             "    ax.scatter(x[-1], y[-1], edgecolor=color, facecolor='none', s=42, label='$t=%.3f$' % t[-1])\n";
  if (t_reuse!=-1) {
    program << "if len(tall) > 0:\n"
               "    ireuse = np.argmin(np.abs(np.array(tall) - " << t_reuse << "))\n"
                                                                               "    ax.scatter(xall[ireuse], yall[ireuse], s=256, marker='+', color='green', label=r'$t_\\mathrm{reuse}=%.3f$' %"
            << t_reuse << ")\n";
  }
  if (t_replan!=-1) {
    program << "if len(tall) > 0:\n"
               "    ireplan = np.argmin(np.abs(np.array(tall) - " << t_replan << "))\n"
                                                                                 "    ax.scatter(xall[ireplan], yall[ireplan], s=256, marker='+', color='magenta', label=r'$t_\\mathrm{replan}=%.3f$' % "
            << t_replan << ")\n";
  }
  program << "ax.legend(loc='best', fontsize=8)\n"
             "ax.set_xlabel('$x$ [m]')\n"
             "ax.set_ylabel('$y$ [m]')\n"
             "ax.set_aspect('equal', 'datalim')\n"
             "fig.suptitle('%d segment(s)' % len(data['segments']))\n"
             "plt.show()\n";
  program.flush();
  std::ostringstream cmd;
  cmd << "python " << program_filename.str() << " &";
  system(cmd.str().c_str());
}

std::vector<std::vector<double>> Trajectory::decompose(double dt_extension) {
  vector<vector<double>> next_xy_vals = {{},
                                         {}};
  for (double t = 0; t < t_max() + dt_extension; t += .02) {
    WorldPose pose = world(t);
    next_xy_vals[0].push_back(pose.x);
    next_xy_vals[1].push_back(pose.y);
  }
  return next_xy_vals;
}

int Trajectory::num_segments() {
  return segments.size();
}

TrajectorySegment::TrajectorySegment(double t0, double DT, FullState begin, FullState end)
    : t_offset(-t0), DT(DT), pt(JMT(begin.s.y, begin.s.yp, begin.s.ypp,
                                    end.s.y, end.s.yp, end.s.ypp,
                                    begin.d.y, begin.d.yp, begin.d.ypp,
                                    end.d.y, end.d.yp, end.d.ypp,
                                    DT)) {}

FrenetPose TrajectorySegment::operator()(double t) {
  double x = remap(t);
  vector<double> sd = pt(x);
  double sp = pt.paths[0].derivative(x);
  double dp = pt.paths[1].derivative(x);

  return {.s=sd[0], .d=sd[1], .yaw=atan2(dp, sp)};
}

double TrajectorySegment::remap(double t) {
  return t + t_offset;
}

double TrajectorySegment::s_derivative(double t, int order) {
  return pt.paths[0].derivative(remap(t), order);
}

double TrajectorySegment::d_derivative(double t, int order) {
  return pt.paths[1].derivative(remap(t), order);
}

