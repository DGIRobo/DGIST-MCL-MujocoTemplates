#include <stdio.h>
#include <math.h>

#include "nlopt.h"

/* Optimization problem

\begin{align}
\min\limits_{x}  f(x) &= a1*x_1^2 + a2*x_2^2 + a3*x_3^2 + a4*x_4^2 + a5*x_5^2 \nonumber \\

x is decision variables.

subject to

Bounds on x
-inf <= x2, x4, x5 <= inf
0.3 <= x1 <= inf
-inf<= x3 <= 5

\mbox{subject to:} & \nonumber \\
Equality constraints
x_1+x_2+x_3 &= ceq_1 \nonumber \\
x_3^2+x_4  &= ceq_2 \nonumber \\

Inequality constraints
x_4^2 + x_5^2 &<= cin_1 \nonumber
\end{align}

Parameters
a1 = a2 = a3 = a4 = a5 = 1;
ceq_1 = 5;
ceq_2 = 2;
cin_1 = 5;

Solution is
1.7736
  1.7736
  1.4527
 -0.1104
 -0.0000

 Cost = 8.414
*/

typedef struct {
    double a[5];
}mycost_data;

typedef struct {
    double ceq_1;
    double ceq_2;
}myequalityconstraints_data;

typedef struct {
    double cin_1;
}myinequalityconstraints_data;


double mycost(unsigned n, const double *x, double *grad, void *costdata)
{
    // define cost function: f(x) &= a1*x_1^2 + a2*x_2^2 + a3*x_3^2 + a4*x_4^2 + a5*x_5^2
    mycost_data *data = (mycost_data *) costdata;

    int i;
    double a[5]={0};
    for (i=0;i<n;i++)
      a[i] = data->a[i];

    double cost = 0;
    for (i=0;i<n;i++)
      cost += a[i]*x[i]*x[i];

    return cost;
}


double myequalityconstraints(unsigned m, double *result, unsigned n,
                             const double *x,  double *grad,
                             void *equalitydata)
{
    // define equality constraints:
    //x_1+x_2+x_3 = ceq_1 = 5
    //x_3^2+x_4 = ceq_2 = 2
    myequalityconstraints_data *data = (myequalityconstraints_data *) equalitydata;

    double c1 = data->ceq_1;
    double c2 = data->ceq_2;
    double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
    result[0] = x1+x2+x3-c1; //5;
    result[1] = x3*x3+x4-c2; //2;
 }

 double myinequalityconstraints(unsigned m, double *result, unsigned n,
                                const double *x,  double *grad,
                                void* inequalitydata)
 {
     // define inequality constraints: 
     //x_4^2 + x_5^2 &<= cin_1
     myinequalityconstraints_data *data = (myinequalityconstraints_data *) inequalitydata;

     double c1 = data->cin_1;
     double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
     result[0] = x4*x4+x5*x5-c1; //5;
  }


void main()
{
int i;
nlopt_opt opt;

//establish sizes
unsigned n = 5; //number of decision variables: x1, x2, x3, x4, x5
unsigned m_eq = 2; //number of equality constraints
// x_1+x_2+x_3 &= ceq_1
// x_3^2+x_4  &= ceq_2
unsigned m_in = 1; //number of inequality constraints
// x_4^2 + x_5^2 &<= cin_1

//bounds for decision variables
double lb[] = { 0.3, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL }; /* lower bounds */
double ub[] = { HUGE_VAL, HUGE_VAL, 5, HUGE_VAL, HUGE_VAL }; /* upper bounds */
// 0.3 <= x1 <= inf
// -inf <= x2 <= inf
// -inf <= x3 <= 5
// -inf <= x4 <= inf
// -inf <= x5 <= inf

//Set the algorithm and dimensionality
//G,L = global/local
//D,N = derivative / no derivative
opt = nlopt_create(NLOPT_LN_COBYLA, n); /* algorithm and dimensionality: set Local No derivative */

//Set the lower and upper bounds
nlopt_set_lower_bounds(opt, lb);
nlopt_set_upper_bounds(opt, ub);

//Set up cost
//f(x) &= x_1^2 + x_2^2 + x_3^2 + x_4^2 + x_5^2
mycost_data costdata;
for (i=0;i<n;i++)
  costdata.a[i]=1;
nlopt_set_min_objective(opt, mycost, &costdata);

//set up equality constraint
//x_1+x_2+x_3 = ceq_1 = 5
//x_3^2+x_4 = ceq_2 = 2
double tol_eq[]={1e-8,1e-8};
myequalityconstraints_data equalitydata;
equalitydata.ceq_1 = 5;
equalitydata.ceq_2 = 2;
nlopt_add_equality_mconstraint(opt, m_eq, myequalityconstraints, &equalitydata, tol_eq);

//set up inequality constraint
//x_4^2 + x_5^2 &<= cin_1
double tol_in[]={1e-8};
myinequalityconstraints_data inequalitydata;
inequalitydata.cin_1 = 5;
nlopt_add_inequality_mconstraint(opt, m_in, myinequalityconstraints,&inequalitydata, tol_in);

//
nlopt_set_xtol_rel(opt, 1e-4);
double x[] = { 1, 1, 1, 2, 1 };  // initial guess
double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
if (nlopt_optimize(opt, x, &minf) < 0) {
    printf("nlopt failed!\n");
}
else {
    printf("found minimum at f(%g,%g,%g,%g,%g) = %0.10g\n", x[0], x[1],x[2], x[3],x[4], minf);
}

nlopt_destroy(opt);
}
