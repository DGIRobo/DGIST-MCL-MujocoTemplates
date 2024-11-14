#include <stdio.h>
#include <math.h>

#include "nlopt.h"

double x_target = 5;
double z_target = 2.1;

double x_end;
double z_end;

extern mjModel* m;
extern mjData* d;


void simulator(double Xin[3], double Xout[2])
{
  double v, theta, time_of_flight;

  v = Xin[0];
  theta = Xin[1];
  time_of_flight = Xin[2];

  d->qvel[0] = v * cos(theta); // calculate x-direction initial velocity and input at mujoco simulation
  d->qvel[2] = v * sin(theta); // calculate z-direction initial velocity and input at mujoco simulation

  while (d->time < time_of_flight)
  {
    mj_step(m, d); // calculate the mujoco simulation without graphical showing during 2sec after starting simulation
  }

  //printf("time: %f, x-pos: %f, z-pos: %f \n", d->time, d->qpos[0], d->qpos[2]); // after 2sec from starting simulation print ball's position at 2sec
  // if z-pos become around 1, this means the ball is one the ground. Because the ball's radius is 1.

  Xout[0] = d->qpos[0]; // return ball's x-position after flight time
  Xout[1] = d->qpos[2]; // return ball's z-position after flight time

  mj_resetData(m, d);
}


double mycost(unsigned n, const double *x, double *grad, void *costdata)
{
    double Xout[2] = {0};
    simulator(x,Xout);

    x_end = Xout[0];
    z_end = Xout[1];

    double cost = x[2]; // hitting time after starting simulation

    return cost;
}


double myequalityconstraints(unsigned m, double *result, unsigned n,
                             const double *x,  double *grad,
                             void *equalitydata)
{
    result[0] = x_end - x_target; // 5
    result[1] = z_end - z_target; // 2
 }


void optimize_ic(double Xin[3]) // initial guess
{
int i;
nlopt_opt opt;

//establish sizes
unsigned n = 3; //number of decision variables
unsigned m_eq = 2; //number of equality constraints
//unsigned m_in = 0; //number of inequality constraints

//bounds for decision variables
// v, theta, t
double lb[] = { 0.1, 0.1, 0.1 }; /* lower bounds */
double ub[] = { HUGE_VAL, 3.14/2 - 0.1, HUGE_VAL }; /* lower bounds */

//Set the algorithm and dimensionality
//L,G = global/local
//D,N = derivative / no derivative
opt = nlopt_create(NLOPT_LN_COBYLA, n); /* algorithm and dimensionality */

//Set the lower and upper bounds
nlopt_set_lower_bounds(opt, lb);
nlopt_set_upper_bounds(opt, ub);

//Set up cost
nlopt_set_min_objective(opt, mycost, NULL);

//set up equality constraint
double tol_eq[]={1e-4,1e-4};
nlopt_add_equality_mconstraint(opt, m_eq, myequalityconstraints, NULL, tol_eq);

nlopt_set_xtol_rel(opt, 1e-4);
double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
if (nlopt_optimize(opt, Xin, &minf) < 0) {
    printf("nlopt failed!\n");
}
else {
    printf("found minimum at f(%g,%g,%g) = %0.10g\n", Xin[0], Xin[1],Xin[2], minf);
}

nlopt_destroy(opt);
}
