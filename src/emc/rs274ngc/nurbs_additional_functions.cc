/********************************************************************

   Author: Manfredi Leto (Xemet)
   License: GPL Version 2
   System: Linux

   Copyright (c) 2009 All rights reserved.

 ********************************************************************/

/* Those functions are needed to calculate NURBS points G_5_2/G_5_3 */

#include <math.h>
#include <algorithm>
#include "canon.hh"

static void unit(NURBS_PLANE_POINT &p)
	{
	double h = hypot(p.NURBS_X, p.NURBS_Y);
	if(h != 0)
		{
		p.NURBS_X/=h;
		p.NURBS_Y/=h;
		}
	}

std::vector<unsigned int> nurbs_G5_knot_vector_creator(unsigned int n, unsigned int k)
	{

	unsigned int i;
	std::vector<unsigned int> knot_vector;
	for(i=0; i<=n+k; i++)
		{
		if(i < k)
			{
			knot_vector.push_back(0);
			}
		else
			if(i >= k && i <= n)
				{
				knot_vector.push_back(i - k + 1);
				}
			else
				{
				knot_vector.push_back(n - k + 2);
				}
		}
	return knot_vector;

	}

double nurbs_G5_Nmix(unsigned int i, unsigned int k, double u,	std::vector<unsigned int> knot_vector)
	{
	if(k == 1)
		{
		if((u >= knot_vector[i]) && (u <= knot_vector[i+1]))
			{
			return 1;
			}
		else
			{
			return 0;
			}
		}
	else
		if(k > 1)
			{
			if((knot_vector[i+k-1]-knot_vector[i] == 0) &&
					(knot_vector[i+k]-knot_vector[i+1] != 0))
				{
				return ((knot_vector[i+k] - u)*nurbs_G5_Nmix(i+1,k-1,u,knot_vector))/
							 (knot_vector[i+k]-knot_vector[i+1]);
				}
			else
				if((knot_vector[i+k]-knot_vector[i+1] == 0) &&
						(knot_vector[i+k-1]-knot_vector[i] != 0))
					{
					return ((u - knot_vector[i])*nurbs_G5_Nmix(i,k-1,u,knot_vector))/
								 (knot_vector[i+k-1]-knot_vector[i]);
					}
				else
					if((knot_vector[i+k-1]-knot_vector[i] == 0) &&
							(knot_vector[i+k]-knot_vector[i+1] == 0))
						{
						return 0;
						}
					else
						{
						return ((u - knot_vector[i])*nurbs_G5_Nmix(i,k-1,u,knot_vector))/
									 (knot_vector[i+k-1]-knot_vector[i]) + ((knot_vector[i+k] - u)*
											 nurbs_G5_Nmix(i+1,k-1,u,knot_vector))/(knot_vector[i+k]-knot_vector[i+1]);
						}
			}
		else
			{
			return -1;
			}
	}



double nurbs_G5_Rden(double u, unsigned int k,	std::vector<NURBS_CONTROL_POINT> nurbs_control_points, std::vector<unsigned int> knot_vector)
	{
	unsigned int i;
	double d = 0.0;
	for(i=0; i<(nurbs_control_points.size()); i++)
		{
		d = d + nurbs_G5_Nmix(i,k,u,knot_vector)*nurbs_control_points[i].NURBS_W;
		}
	return d;
	}

NURBS_PLANE_POINT nurbs_G5_point(double u, unsigned int k,	std::vector<NURBS_CONTROL_POINT> nurbs_control_points, std::vector<unsigned int> knot_vector)
	{
	unsigned int i;
	NURBS_PLANE_POINT point;
	point.NURBS_X = 0;
	point.NURBS_Y = 0;
	for(i=0; i<(nurbs_control_points.size()); i++)
		{
		point.NURBS_X = point.NURBS_X + nurbs_control_points[i].NURBS_X*nurbs_G5_Nmix(i,k,u,knot_vector)*nurbs_control_points[i].NURBS_W/nurbs_G5_Rden(u,k,nurbs_control_points,knot_vector);
		point.NURBS_Y = point.NURBS_Y + nurbs_control_points[i].NURBS_Y*nurbs_G5_Nmix(i,k,u,knot_vector)*nurbs_control_points[i].NURBS_W/nurbs_G5_Rden(u,k,nurbs_control_points,knot_vector);
		}
	return point;
	}

#define DU (1e-5)
NURBS_PLANE_POINT nurbs_G5_tangent(double u, unsigned int k, std::vector<NURBS_CONTROL_POINT> nurbs_control_points, std::vector<unsigned int> knot_vector)
	{
	unsigned int n = nurbs_control_points.size() - 1;
	double umax = n - k + 2;
	double ulo = std::max(0.0, u-DU), uhi = std::min(umax, u+DU);
	NURBS_PLANE_POINT P1 = nurbs_G5_point(ulo, k, nurbs_control_points, knot_vector);
	NURBS_PLANE_POINT P3 = nurbs_G5_point(uhi, k, nurbs_control_points, knot_vector);
	NURBS_PLANE_POINT r = {(P3.NURBS_X - P1.NURBS_X) / (uhi-ulo), (P3.NURBS_Y - P1.NURBS_Y) / (uhi-ulo)};
	unit(r);
	return r;
	}


/*********************************************************************/
// Those functions are needed to calculate NURBS points   G_6_2
// Author: Drago Stefano
/*********************************************************************/

static void unit_(NURBS_PLANE_POINT &p)
	{
	double h = hypot(p.NURBS_X, p.NURBS_Y);
	if(h != 0)
		{
		p.NURBS_X/=h;
		p.NURBS_Y/=h;
		}
	}

std::vector<double> nurbs_g6_knot_vector_creator(unsigned int n, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points)
	{
	unsigned int i;
	std::vector<double> knot_vector_;
	for(i=0; i<=n+k; i++)
		{
		knot_vector_.push_back(nurbs_control_points[i].NURBS_K);
		}
	return knot_vector_;
	}

double nurbs_G6_Nmix(unsigned int i, unsigned int k, double u, std::vector<double> knot_vector_)
	{
	if(k == 1)
		{
		if((u >= knot_vector_[i]) && (u <= knot_vector_[i+1]))
			{
			return 1;
			}
		else
			{
			return 0;
			}
		}
	else
		{
		if(k > 1)
			{
			if((knot_vector_[i+k-1]-knot_vector_[i] == 0) && (knot_vector_[i+k]-knot_vector_[i+1] != 0))
				{
				return ((knot_vector_[i+k] - u)*nurbs_G6_Nmix(i+1,k-1,u,knot_vector_))/ (knot_vector_[i+k]-knot_vector_[i+1]);
				}
			else
				{
				if((knot_vector_[i+k]-knot_vector_[i+1] == 0) && (knot_vector_[i+k-1]-knot_vector_[i] != 0))
					{
					return ((u - knot_vector_[i])*nurbs_G6_Nmix(i,k-1,u,knot_vector_))/ (knot_vector_[i+k-1]-knot_vector_[i]);
					}
				else
					{
					if((knot_vector_[i+k-1]-knot_vector_[i] == 0) && (knot_vector_[i+k]-knot_vector_[i+1] == 0))
						{
						return 0;
						}
					else
						{
						return ((u - knot_vector_[i])*nurbs_G6_Nmix(i,k-1,u,knot_vector_))/(knot_vector_[i+k-1]-knot_vector_[i]) + ((knot_vector_[i+k] - u)*nurbs_G6_Nmix(i+1,k-1,u,knot_vector_))/(knot_vector_[i+k]-knot_vector_[i+1]);
						}
					}
				}
			}
		else
			{
			return 0;
			}
		}
	}

//A6 è la matrice dove sono memorizzati i valori delle funzioni di base Ni,p(u) per dato valore di u
//A6 ist die Matrix, in der die Werte der Basisfunktionen Ni,p(u) für einen gegebenen Wert von u gespeichert sind
std::vector< std::vector<double> > nurbs_G6_Nmix_creator(double u, unsigned int k, double n, std::vector<double> knot_vector)
	{
	std::vector< std::vector<double> > A6; // array
	//printf("u: %f k: %d n: %f (F: %s L: %d)\n", u, k, n, __FILE__, __LINE__);

	for(unsigned int i=0; i<n; i++)
		{
		A6.push_back(std::vector<double>(k+1,0));
		
		}
	for (long unsigned int i=0; i < A6.size(); i++) {
		//printf("  i: %ld A6: %f %f (F: %s L: %d)\n", i, A6[0][i], A6[1][i], __FILE__, __LINE__);
	}
		
	for(unsigned int p=1; p<=k; ++p)
		{
		for(unsigned int j=0; j<n; ++j)
			{
			A6[j][p]= nurbs_G6_Nmix(j, p, u, knot_vector);
			//printf("  j: %d p: %d A6: %f (F: %s L: %d)\n", j, p, A6[j][p], __FILE__, __LINE__);
			}
		}
	return A6;
	}

double nurbs_Rden_(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector)
	{
	unsigned int i;
	double d = 0.0;
	for(i=0; i<(nurbs_control_points.size()-k); i++)
		{
		d = d + nurbs_G6_Nmix(i,k,u,knot_vector)*nurbs_control_points[i].NURBS_R;
		}
	return d;
	}

// nurbs_G6_point is unused?
NURBS_PLANE_POINT nurbs_G6_point(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector)
	{
	unsigned int i;
	NURBS_PLANE_POINT point;
	point.NURBS_X = 0;
	point.NURBS_Y = 0;
	for(i=0; i<(nurbs_control_points.size()-k); i++)
		{
		point.NURBS_X = point.NURBS_X + nurbs_control_points[i].NURBS_X*nurbs_G6_Nmix(i,k,u,knot_vector)	*nurbs_control_points[i].NURBS_R/nurbs_Rden_(u,k,nurbs_control_points,knot_vector);
		point.NURBS_Y = point.NURBS_Y + nurbs_control_points[i].NURBS_Y*nurbs_G6_Nmix(i,k,u,knot_vector)	*nurbs_control_points[i].NURBS_R/nurbs_Rden_(u,k,nurbs_control_points,knot_vector);
		}
	return point;
	}

double nurbs_Rdenx(double /*u*/, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> /*knot_vector*/, std::vector< std::vector<double> > A6)
	{
	unsigned int i;
	double d = 0.0;
	for(i=0; i<(nurbs_control_points.size()-k); i++)
		{
		d = d + A6[i][k]*nurbs_control_points[i].NURBS_R;
		}
	return d;
	}

// Algoritmo Cox-Boor
NURBS_PLANE_POINT nurbs_G6_pointx(double u, unsigned int k,	std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector, std::vector< std::vector<double> > A6)
	{
	unsigned int i;
	NURBS_PLANE_POINT point;
	point.NURBS_X = 0;
	point.NURBS_Y = 0;
	for(i=0; i<(nurbs_control_points.size()-k); i++)
		{
		point.NURBS_X = point.NURBS_X + nurbs_control_points[i].NURBS_X*(A6[i][k])*nurbs_control_points[i].NURBS_R/nurbs_Rdenx(u,k,nurbs_control_points,knot_vector,A6);
		point.NURBS_Y = point.NURBS_Y + nurbs_control_points[i].NURBS_Y*(A6[i][k])*nurbs_control_points[i].NURBS_R/nurbs_Rdenx(u,k,nurbs_control_points,knot_vector,A6);
		}
	return point;
	}

//ALGORITMO DE_BOOR
//////La funzione nurbs_G6_point_x calcola un punto C(u) della curva NURBS per dato valore del parametro u
NURBS_PLANE_POINT nurbs_G6_point_x(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector)   // è una funzione
	{
	NURBS_PLANE_POINT pointx;
	pointx.NURBS_X = 0;
	pointx.NURBS_Y = 0;
	double h,p, ai_r;
	p=k-1;
	int s=0;
	h=p-s;
	int m=-1, kk=-1;

	for(unsigned j=0; j<knot_vector.size(); ++j)
		{
		if((knot_vector[j]> u) && (kk == -1))
			{
			kk=j-1;
			}
		if((m ==-1) && (u == knot_vector[knot_vector.size()-1]))
			{
			m=10;
			}
		if(u == knot_vector[j])
			{
			s=1;
			h=p-s;
			}
		}
	if(m == 10)
		{
		pointx.NURBS_X = nurbs_control_points[nurbs_control_points.size()-k-1].NURBS_X;
		pointx.NURBS_Y = nurbs_control_points[nurbs_control_points.size()-k-1].NURBS_Y;
		}
	else
		{
		NURBS_CONTROL_POINT CPP;
		std::vector<NURBS_CONTROL_POINT> nurbs_control_points_a;
		nurbs_control_points_a.clear();
		for(int j=0; j<=(kk-s); ++j)
			{
			CPP.NURBS_X = nurbs_control_points[j].NURBS_X*nurbs_control_points[j].NURBS_R;
			CPP.NURBS_Y = nurbs_control_points[j].NURBS_Y*nurbs_control_points[j].NURBS_R;
			CPP.NURBS_W = nurbs_control_points[j].NURBS_R;
			nurbs_control_points_a.push_back(CPP);
			// printf(" P.X%lf P.Y%lf\n", CPP.X, CPP.Y);
			}

		double Pi_r_X, Pi_r_Y, Pi_r_W; // warning: ‘Pi_r_X, Pi_r_Y, Pi_r_W’ may be used uninitialized
		Pi_r_X=0.0; Pi_r_Y=0.0; Pi_r_W=0.0;	// jf
		NURBS_CONTROL_POINT CPPx;
		std::vector<NURBS_CONTROL_POINT> nurbs_control_points_x;
		nurbs_control_points_x.clear();
		//for (unsigned int j=0; j<=kk-s; ++j){ // vettore copia  // warning: comparison of integer expressions of different signedness
		for(int j=0; j<=kk-s; ++j)   // vettore copia
			{
			CPPx.NURBS_X = nurbs_control_points_a[j].NURBS_X;
			CPPx.NURBS_Y = nurbs_control_points_a[j].NURBS_Y;
			CPPx.NURBS_W = nurbs_control_points_a[j].NURBS_W;
			nurbs_control_points_x.push_back(CPPx);
			}

		for(unsigned r=1; r<=h; ++r)
			{
			for(int j=0; j<=kk-s; ++j)
				{
				nurbs_control_points_x[j].NURBS_X=nurbs_control_points_a[j].NURBS_X;
				nurbs_control_points_x[j].NURBS_Y=nurbs_control_points_a[j].NURBS_Y;
				nurbs_control_points_x[j].NURBS_W=nurbs_control_points_a[j].NURBS_W;
				}
			for(int i=(kk-p+r); i<=kk-s; ++i)
				{
				ai_r=(u-knot_vector[i])/(knot_vector[i+p-r+1]-knot_vector[i]);
				Pi_r_X= (1-ai_r)*nurbs_control_points_x[i-1].NURBS_X+ ai_r * nurbs_control_points_x[i].NURBS_X;
				nurbs_control_points_a[i].NURBS_X=Pi_r_X;
				Pi_r_Y=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_Y)+(ai_r*nurbs_control_points_x[i].NURBS_Y);
				nurbs_control_points_a[i].NURBS_Y=Pi_r_Y;
				Pi_r_W=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_W)+(ai_r*nurbs_control_points_x[i].NURBS_W);
				nurbs_control_points_a[i].NURBS_W=Pi_r_W;
				}
			}
		pointx.NURBS_X = Pi_r_X/Pi_r_W;
		pointx.NURBS_Y = Pi_r_Y/Pi_r_W;
		// printf("p1x=%lf, p1y=%lf \n", pointx.X, pointx.Y);
		}
	return pointx;
	}

//ALGORITMO DI SUDDIVISIONE NURBS. La CURVA è suddivisa in due segmenti.La funzione calcola i punti di controllo del primo sgmento NURBS
std::vector<double> nurbs_G6_new_control_point_nurbs1(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points,	std::vector<double> knot_vector)
	{
	std::vector<NURBS_CONTROL_POINT> nurbs_control_points_a;
	std::vector<NURBS_CONTROL_POINT> nurbs_control_points_x;
	std::vector<double> new_control_pointx;
	std::vector<double> new_control_pointx2;
	double h,p, ai_r;
	p=k-1;
	// printf("p=%lf\n",p);
	int s=1;
	h=p-s;
	// int m=-1; //warning: unused variable ‘m’
	int kk=-1;
	for(unsigned j=0; j<knot_vector.size(); ++j)
		{
		if((knot_vector[j]> u) && (kk == -1))
			{
			kk=j-1;
			}
		}
	for(unsigned i=0; i<(kk-p+1);  ++i)
		{
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_X);
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_Y);
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_R);
		}

	NURBS_CONTROL_POINT CPP;
	nurbs_control_points_a.clear();
	//for (unsigned int j=0; j<=(kk-s); ++j){	//warning: comparison of integer expressions of different signedness
	for(int j=0; j<=(kk-s); ++j)
		{
		CPP.NURBS_X = nurbs_control_points[j].NURBS_X*nurbs_control_points[j].NURBS_R;
		CPP.NURBS_Y = nurbs_control_points[j].NURBS_Y*nurbs_control_points[j].NURBS_R;
		CPP.NURBS_W = nurbs_control_points[j].NURBS_R;
		nurbs_control_points_a.push_back(CPP);
		}

	double Pi_r_X, Pi_r_Y, Pi_r_W;
	NURBS_CONTROL_POINT CPPx;
	nurbs_control_points_x.clear();
	//for (unsigned int j=0; j<=kk-s; ++j){ //costruisce un vettore copia //warning: comparison of integer expressions of different signedness
	for(int j=0; j<=kk-s; ++j)   //costruisce un vettore copia
		{
		CPPx.NURBS_X = nurbs_control_points_a[j].NURBS_X;
		CPPx.NURBS_Y = nurbs_control_points_a[j].NURBS_Y;
		CPPx.NURBS_W = nurbs_control_points_a[j].NURBS_W;
		nurbs_control_points_x.push_back(CPPx);
		}
	int c2;
	for(unsigned r=1; r<=h; ++r)
		{
		//printf("r=%d\n",r);
		//for (unsigned i=(kk-p+r); r<=kk-s; ++r){   kk-p=0
		//for (unsigned i=0; i<=r; ++i){

		//for (unsigned int j=0; j<=kk-s; ++j){	//warning: comparison of integer expressions of different signedness
		for(int j=0; j<=kk-s; ++j)
			{
			nurbs_control_points_x[j].NURBS_X=nurbs_control_points_a[j].NURBS_X;
			nurbs_control_points_x[j].NURBS_Y=nurbs_control_points_a[j].NURBS_Y;
			nurbs_control_points_x[j].NURBS_W=nurbs_control_points_a[j].NURBS_W;
			}
		c2=0;
		//for (unsigned i=(kk-p+r); i<=kk-s; ++i){ //warning: comparison of integer expressions of different signedness
		for(int i=(kk-p+r); i<=kk-s; ++i)
			{
			//printf("P%d,%d=\n", i,r);
			ai_r=(u-knot_vector[i])/(knot_vector[i+p-r+1]-knot_vector[i]);
			//printf("ai_r=%lf\n",ai_r);
			//Pi_r=(1-ai_r)* Pi-1,r-1+ Pi,r-1

			Pi_r_X= (1-ai_r)*nurbs_control_points_x[i-1].NURBS_X+ ai_r * nurbs_control_points_x[i].NURBS_X;
			nurbs_control_points_a[i].NURBS_X=Pi_r_X;

			Pi_r_Y=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_Y)+(ai_r*nurbs_control_points_x[i].NURBS_Y);
			nurbs_control_points_a[i].NURBS_Y=Pi_r_Y;

			Pi_r_W=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_W)+(ai_r*nurbs_control_points_x[i].NURBS_W);
			nurbs_control_points_a[i].NURBS_W=Pi_r_W;
			//printf("(P%d,%d  P%d,%d)\n", i-1,r-1, i,r-1);
			//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W\n",r,i,Pi_r_X,r,i, Pi_r_Y,r,i,Pi_r_W);

			Pi_r_X= Pi_r_X/ Pi_r_W;
			Pi_r_Y= Pi_r_Y/ Pi_r_W;
			//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W=%lf\n",i,r,Pi_r_X,i,r, Pi_r_Y,i,r,Pi_r_W);

			if(c2 == 0)
				{
				//printf("c2=%d\n",c2);
				//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W=%lf\n",i,r,Pi_r_X,i,r, Pi_r_Y,i,r,Pi_r_W);
				new_control_pointx.push_back(Pi_r_X);
				new_control_pointx.push_back(Pi_r_Y);
				new_control_pointx.push_back(Pi_r_W);
				c2=-1;
				}
			}
		}
	return new_control_pointx;
	}

//La funzione calcola i punti di controllo del secondo segmento NURBS
// è una funzione
std::vector<double> nurbs_G6_new_control_point_nurbs2(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points,	std::vector<double> knot_vector) 
	{
	std::vector<NURBS_CONTROL_POINT> nurbs_control_points_a;
	std::vector<NURBS_CONTROL_POINT> nurbs_control_points_x;
	std::vector<double> new_control_pointx;
	std::vector<double> new_control_pointx2;
	double h,p, ai_r;
	p=k-1;
	// printf("p=%lf\n",p);
	int s=1;

	h=p-s;
	//int m=-1;	//warning: unused variable ‘m’
	int kk=-1;

	for(unsigned j=0; j<knot_vector.size(); ++j)
		{
		if((knot_vector[j]> u) && (kk == -1))
			{
			kk=j-1; //deve ritornare l'indice del passo precedente, il quale individua Pk
			// printf("j=%d, knot=%d\n",j,knot_vector[j]);
			//break;
			}
		}

	// printf("kk=%d\n",kk);
	NURBS_CONTROL_POINT CPP;
	nurbs_control_points_a.clear();
	//printf(" Array  punti di controllo necessare per inserire il knot e calocolare il punto C(u)\n", CPP.X, CPP.Y);
	//for (unsigned int j=0; j<=(kk-s); ++j){ //warning: comparison of integer expressions of different signedness
	for(int j=0; j<=(kk-s); ++j)
		{
		//printf("j=%d\n",j);
		CPP.NURBS_X = nurbs_control_points[j].NURBS_X*nurbs_control_points[j].NURBS_R;
		CPP.NURBS_Y = nurbs_control_points[j].NURBS_Y*nurbs_control_points[j].NURBS_R;
		CPP.NURBS_W = nurbs_control_points[j].NURBS_R;
		nurbs_control_points_a.push_back(CPP);

		// printf(" P.X%lf P.Y%lf R%lf\n", CPP.X, CPP.Y, CPP.W);
		}

	double Pi_r_X, Pi_r_Y, Pi_r_W;
	NURBS_CONTROL_POINT CPPx;
	nurbs_control_points_x.clear();
	//for (unsigned int j=0; j<=kk-s; ++j){ //costruisce un vettore copia //warning: comparison of integer expressions of different signedness
	for(int j=0; j<=kk-s; ++j)   //costruisce un vettore copia
		{
		CPPx.NURBS_X = nurbs_control_points_a[j].NURBS_X;
		CPPx.NURBS_Y = nurbs_control_points_a[j].NURBS_Y;
		CPPx.NURBS_W = nurbs_control_points_a[j].NURBS_W;
		nurbs_control_points_x.push_back(CPPx);
		}

	for(unsigned r=1; r<=h; ++r)
		{
		//for (unsigned int j=0; j<=kk-s; ++j){      // warning: comparison of integer expressions of different signedness
		for(int j=0; j<=kk-s; ++j)
			{
			nurbs_control_points_x[j].NURBS_X=nurbs_control_points_a[j].NURBS_X;
			nurbs_control_points_x[j].NURBS_Y=nurbs_control_points_a[j].NURBS_Y;
			nurbs_control_points_x[j].NURBS_W=nurbs_control_points_a[j].NURBS_W;
			}

		//for (unsigned i=(kk-p+r); i<=kk-s; ++i){    //  warning: comparison of integer expressions of different signedness
		for(int i=(kk-p+r); i<=kk-s; ++i)
			{
			//printf("P%d,%d=\n", i,r);
			ai_r=(u-knot_vector[i])/(knot_vector[i+p-r+1]-knot_vector[i]);
			//printf("ai_r=%lf\n",ai_r);
			//Pi_r=(1-ai_r)* Pi-1,r-1+ Pi,r-1

			Pi_r_X= (1-ai_r)*nurbs_control_points_x[i-1].NURBS_X+ ai_r * nurbs_control_points_x[i].NURBS_X;
			nurbs_control_points_a[i].NURBS_X=Pi_r_X;

			Pi_r_Y=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_Y)+(ai_r*nurbs_control_points_x[i].NURBS_Y);
			nurbs_control_points_a[i].NURBS_Y=Pi_r_Y;

			Pi_r_W=((1-ai_r)*nurbs_control_points_x[i-1].NURBS_W)+(ai_r*nurbs_control_points_x[i].NURBS_W);
			nurbs_control_points_a[i].NURBS_W=Pi_r_W;
			//printf("(P%d,%d  P%d,%d)\n", i-1,r-1, i,r-1);
			//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W\n",r,i,Pi_r_X,r,i, Pi_r_Y,r,i,Pi_r_W);

			Pi_r_X= Pi_r_X/ Pi_r_W;
			Pi_r_Y= Pi_r_Y/ Pi_r_W;
			//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W=%lf\n",i,r,Pi_r_X,i,r, Pi_r_Y,i,r,Pi_r_W);

			if(i ==kk-s)
				{
				//printf("kk-s=%d\n",kk-s);
				//printf("P%d_%d_X=%lf   P%d_%d_Y=%lf  P%d_%d_W=%lf\n",i,r,Pi_r_X,i,r, Pi_r_Y,i,r,Pi_r_W);
				new_control_pointx2.push_back(Pi_r_X);
				new_control_pointx2.push_back(Pi_r_Y);
				new_control_pointx2.push_back(Pi_r_W);
				}
			}
		}

	for(int j=new_control_pointx2.size()-1; j>=2; j=j-3)
		{
		new_control_pointx.push_back(new_control_pointx2[j-2]);
		new_control_pointx.push_back(new_control_pointx2[j-1]);
		new_control_pointx.push_back(new_control_pointx2[j]);
		}

	/*********************************************************************************************/
	for(unsigned i=kk-s; i<(nurbs_control_points.size()-k);  ++i)   //aggiusta la dimensione -k
		{
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_X);
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_Y);
		new_control_pointx.push_back(nurbs_control_points[i].NURBS_R);
		//printf("X%lf Y%lf W%lf\n", nurbs_control_points[i].X,nurbs_control_points[i].Y,nurbs_control_points[i].W);
		}
	return new_control_pointx;
	}

//Funzione per ridefinire il vettore knot per il segmento 1 e 2  dopo la suddivisione
std::vector<double> nurbs_G6_knot_vector_new_creator_sgment(unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points)
	{
	std::vector<double> knot_vector_sgment;
	int n=nurbs_control_points.size()-1-k;
	//for(int i=0; i<=n+k; i++)	// warning: comparison of integer expressions of different signedness
	for(int i=0; i<=n+(int)k; i++)
		{
		// if(i < k)	// warning: comparison of integer expressions of different signedness
		if(i < (int)k)
			{
			knot_vector_sgment.push_back(0);
			}
		else
			{
			//if(i >= k && i <= n)	// warning: comparison of integer expressions of different signedness
			if(i >= (int)k && i <= n)
				{
				knot_vector_sgment.push_back(i - k + 1);
				}
			else
				{
				knot_vector_sgment.push_back(n - k + 2);
				}
			}
		}
	return knot_vector_sgment;
	}

//Per l'algoritmo DE-BOOR
#define DU (1e-5)
NURBS_PLANE_POINT nurbs_G6_tangent_x(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_)
	{
	NURBS_PLANE_POINT r, P1, P3;
	//unsigned int n = nurbs_control_points.size() -k - 1;	// warning: unused variable ‘n’
	double umax = knot_vector_[knot_vector_.size()-1];
	double ulo = std::max(knot_vector_[0], u-DU), uhi = std::min(umax, u+DU);
	P1 = nurbs_G6_point_x(ulo, k, nurbs_control_points, knot_vector_);
	P3 = nurbs_G6_point_x(uhi, k, nurbs_control_points, knot_vector_);
	r = {(P3.NURBS_X - P1.NURBS_X) / (uhi-ulo), (P3.NURBS_Y - P1.NURBS_Y) / (uhi-ulo)};
	unit_(r);
	double aa=1, bb=0, cc;
	cc= aa/bb; //cc=inf
	if(r.NURBS_X == cc || r.NURBS_Y == cc)
		{
		r.NURBS_X=0;
		r.NURBS_Y=0;
		//printf("knot==u  infinito\n");
		}
	return r;
	}

/******************************************************************************************************************************************/
// La funzione Nderv restituisce la derivata prima della funzione di base rispetto al parametro u
// N'i,k(u)=.....
double Nderv(unsigned int i, unsigned int k, double u, std::vector<double> knot_vector_)
	{
	if(k == 1)
		{
		if((u >= knot_vector_[i]) && (u <= knot_vector_[i+1]))
			{
			return 1;
			}
		else
			{
			return 0;
			}
		}
	else
		{
		if(k > 1)
			{
			if((knot_vector_[i+k-1]-knot_vector_[i] == 0) && (knot_vector_[i+k]-knot_vector_[i+1] != 0))
				{
				return  -((k-1)*nurbs_G6_Nmix(i+1,k-1,u,knot_vector_))/(knot_vector_[i+k]-knot_vector_[i+1]);
				}
			else
				{
				if((knot_vector_[i+k]-knot_vector_[i+1] == 0) && (knot_vector_[i+k-1]-knot_vector_[i] != 0))
					{
					return ((k-1)*nurbs_G6_Nmix(i,k-1,u,knot_vector_))/(knot_vector_[i+k-1]-knot_vector_[i]) ;
					}
				else
					{
					if((knot_vector_[i+k-1]-knot_vector_[i] == 0) && (knot_vector_[i+k]-knot_vector_[i+1] == 0))
						{
						return 0;
						}
					else
						{
						return ((k-1)*nurbs_G6_Nmix(i,k-1,u,knot_vector_))/(knot_vector_[i+k-1]-knot_vector_[i]) - ((k-1)*nurbs_G6_Nmix(i+1,k-1,u,knot_vector_))/(knot_vector_[i+k]-knot_vector_[i+1]);
						}
					}
				}
			}
		else
			{
			return -1;
			}
		}
	}

//**********************************************************************************/
//calcolo della derivata  della funzione di base razionale
double Rderv(double u, unsigned int k, unsigned int i, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_,double f,double Rsk)
	{
	double d = 0.0;
	double Nik,DNik;
	Nik = nurbs_G6_Nmix(i,k,u,knot_vector_);    // calcolo della funzione di base
	DNik = Nderv(i,k,u,knot_vector_); // derivata prima della funzione di basa Ni,k
	d = ((DNik * nurbs_control_points[i].NURBS_R) / Rsk)-(Nik * nurbs_control_points[i].NURBS_R *f / (Rsk*Rsk));
	return d;
	}

// LA funzione Dnurbs_point calcola   la derivata della nurbs C'(u) in u  è una funzione
NURBS_G6_DPLANE_POINT Dnurbs_point(double u, unsigned int k,	std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_)
	{
	double DRik;
	unsigned int i;
	double f = 0.0;
	double Rsk;
	NURBS_G6_DPLANE_POINT Dpoint;
	Dpoint.DX = 0;
	Dpoint.DY = 0;

	//for(int ii=0; ii<(nurbs_control_points.size()-k); ii++)	//warning: comparison of integer expressions of different signedness
	for(long unsigned int ii=0; ii<(nurbs_control_points.size()-k); ii++)
		{
		f = f + Nderv(ii,k,u,knot_vector_)*nurbs_control_points[ii].NURBS_R;
		}
	Rsk= nurbs_Rden_(u,k,nurbs_control_points,knot_vector_);
	for(i=0; i<(nurbs_control_points.size()-k); i++)
		{
		DRik=Rderv(u, k, i, nurbs_control_points, knot_vector_,f,Rsk);
		Dpoint.DX = Dpoint.DX + nurbs_control_points[i].NURBS_X*DRik;
		Dpoint.DY = Dpoint.DY + nurbs_control_points[i].NURBS_Y*DRik;
		}
	return Dpoint;
	}

// Nel vettore sono definiti gli estremi degli intervalli di integrazione necessari a definire l'inversa della funzione lunghezza
std::vector<double> nurbs_interval_span_knot_vector_creator(unsigned int n, unsigned int k, std::vector<double> knot_vector_)
	{
	std::vector<double> span_knot_vector;
	double a, b,c,ac,cb, ac1,ac2, cb1, cb2;
	//for(int i=k-1; i<n+1; i++)    //0-1; 1-2; 2-3; 3-4; span knot // warning: comparison of integer expressions of different signedness
	for(unsigned int i=k-1; i<n+1; i++)    //0-1; 1-2; 2-3; 3-4; span knot
		{
		a = knot_vector_[i];
		b = knot_vector_[i+1];
		c=(a+b)/2;
		ac = (a+c)/2;
		cb = (b+c)/2;
		ac1 = (a+ac)/2;
		ac2 = (ac+c)/2;
		cb1 = (c+cb)/2;
		cb2 = (cb+b)/2;
		span_knot_vector.push_back(a);
		span_knot_vector.push_back(ac1);
		span_knot_vector.push_back(ac);
		span_knot_vector.push_back(ac2);
		span_knot_vector.push_back(c);
		span_knot_vector.push_back(cb1);
		span_knot_vector.push_back(cb);
		span_knot_vector.push_back(cb2);

		if(i == n)
			{
			span_knot_vector.push_back(b);
			}
		}
	return span_knot_vector;
	}

double lderv(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_)
	{
	NURBS_G6_DPLANE_POINT DP1;
	DP1 = Dnurbs_point(u,k,nurbs_control_points,knot_vector_);
	double dl=0;
	dl=sqrt(DP1.DX*DP1.DX+DP1.DY*DP1.DY);
	return dl;
	}

////////////////////////////////////////////////////////////////////////////////////7

//Regola di Simpsn

double Sa1_b1_length_(double a1, double b1, unsigned int k,	std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_)
	{
	double Sa1_b1_,h,c1;
	h=(b1-a1)/2;
	c1=(a1+b1)/2;
	Sa1_b1_=(h/3)*(lderv(a1,k,nurbs_control_points,knot_vector_)+(4*lderv(c1,k,nurbs_control_points,knot_vector_)) + lderv(b1,k,nurbs_control_points, knot_vector_));
	return Sa1_b1_;
	}

// Premesso che ciascun span knot è suddiviso in due in due intervalli,di ciascun intervallo occorre calcolarne la lunghezza, nel vettore sono calcolate è memorizzate le lunghezze realative a ciascun intevallo di integrazione
std::vector<double> nurbs_lenght_vector_creator(unsigned int k,	std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_, std::vector<double> span_knot_vector)
	{
	std::vector<double> lenght_vector;
	double a;
	int n= span_knot_vector.size()-1;
	for(int i=0; i<n; i++)     //0-1; 1-2; 2-3; 3-4; span knot
		{
		a = Sa1_b1_length_(span_knot_vector[i],span_knot_vector[i+1],k, nurbs_control_points,knot_vector_);
		lenght_vector.push_back(a);
		}
	return lenght_vector; // ogni 2 posiziono sono definite le lunghezze dei due intervalli in cui è stato suddiviso ciascun span knot
	}

///////////////////////////////////////////////////////////////////////////////////////

// Funzione per calcolare la lunghezza totale della curva fino all'intervallo di integrazione dello span knot di estremo superiore bj, [aj,bj] dove j individua l'ultimo intervallo di integrazione.
double nurbs_lenght_tot(int j, std::vector<double> /*span_knot_vector*/, std::vector<double> lenght_vector)
	{
	double ltot=0;
	for(int i=0; i<=j; ++i)
		{
		ltot = ltot + lenght_vector[i];
		}
	return ltot;
	}

// Funzione per il calcolo della lunghezza della curva per un qualsiasi valore di u. Tale funzione è stata impiegata per realizzare una verifica
double lenght_l_u(double u, unsigned int k, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_, std::vector<double> span_knot_vector, std::vector<double> lenght_vector)
	{
	double a,b,c,d,e,f,g,h,at=0;
	//double l_u; // warning: ‘l_u’ may be used uninitialized
	double l_u=0.0; 	// jf
	//double lj; // warning: ‘lj’ may be used uninitialized
	double lj=0.0; 	// jf

	if(u==0)
		{
		l_u=0;
		printf("u=%lf  l_u=%lf\n",u,  l_u);
		}
	else
		{
		for(unsigned ii=0; ii<span_knot_vector.size(); ++ii)
			{
			if((span_knot_vector[ii] >= u)&& (at == 0))
				{
				at=at+1;
				int j=ii-1, jj=ii-2;
				if(jj >=0)
					{
					lj = nurbs_lenght_tot(jj, span_knot_vector,lenght_vector);
					}

				double a1, b1, b2, a1b1, b1b2, x, q, w, m;
				a1 = span_knot_vector[j];
				b2 = u;
				b1 = (a1+b2)/2;
				a1b1 = (a1+b1)/2;
				b1b2 = (b2+b1)/2;
				x = (a1+a1b1)/2;
				q = (a1b1+b1)/2;
				m = (b1+b1b2)/2;
				w = (b1b2+b2)/2;

				a = Sa1_b1_length_(a1,x,k, nurbs_control_points,knot_vector_);
				b = Sa1_b1_length_(x,a1b1,k, nurbs_control_points,knot_vector_);
				c = Sa1_b1_length_(a1b1,q,k, nurbs_control_points,knot_vector_);
				d = Sa1_b1_length_(q,b1,k, nurbs_control_points,knot_vector_);
				e = Sa1_b1_length_(b1,m,k, nurbs_control_points,knot_vector_);
				f = Sa1_b1_length_(m,b1b2,k, nurbs_control_points,knot_vector_);
				g = Sa1_b1_length_(b1b2,w,k, nurbs_control_points,knot_vector_);
				h = Sa1_b1_length_(w,b2,k, nurbs_control_points,knot_vector_);
				l_u = lj + a + b + c + d + e + f + g + h;

				//printf("u=%lf  a+b=%lf lj=%lf  l_u=%lf\n",u, a+b,lj, l_u);
				break;
				}
			}
		}
	return l_u;
	}

//vettore dove sono memorizzate le derivate della funzione inversa della funzione lunghezza in corrispondenza u'j(lj) degli estremi degli intervalli di integrazione
std::vector<double> nurbs_Du_span_knot_vector_creator(unsigned int k,	std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, std::vector<double> knot_vector_, std::vector<double> span_knot_vector)
	{
	std::vector<double> Du_span_knot_vector;
	for(unsigned ii=0; ii<span_knot_vector.size(); ++ii)
		{
		double Dl_u = lderv(span_knot_vector[ii],k,nurbs_control_points,knot_vector_);
		double Du_l = 1/Dl_u;
		Du_span_knot_vector.push_back(Du_l);
		}
	return Du_span_knot_vector;
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////777
//In questo vettore sono memorizzati i coefficienti del polinomio di terzo grado impiegato per approssimare la funzione inversa della lunghezza u_l per ogni intervallo di intgrazione; ogni 6 posizioni sono memorizzati i valori relativi a ciascun intervallo di integrazione a_b_c_d_lj_lj1
std::vector<double> nurbs_costant_crator(std::vector<double> span_knot_vector, std::vector<double> lenght_vector, std::vector<double> Du_span_knot_vector)
	{
	std::vector<double> nurbs_costant;
	double a,b,c,d,a1,b1,lj1,ll,lj;

	a = span_knot_vector[0]; //uj
	a1 = span_knot_vector[1];//uj+1
	b = Du_span_knot_vector[0];//u'j
	b1 = Du_span_knot_vector[1];//u'j+1
	lj1 = nurbs_lenght_tot(0, span_knot_vector, lenght_vector);//lj+1
	lj = 0;//lj
	ll = pow((lj1), 2);
	c = (((a1-a)-(b*lj1))/ ll);
	d = ((b1 + b)/ll) - (2*(a1-a)/ (pow((lj1), 3)));

	nurbs_costant.push_back(a);
	nurbs_costant.push_back(b);
	nurbs_costant.push_back(c);
	nurbs_costant.push_back(d);
	nurbs_costant.push_back(lj);
	nurbs_costant.push_back(lj1);

	for(unsigned ii=1; ii< lenght_vector.size(); ++ii)
		{
		a = span_knot_vector[ii]; //uj
		a1 = span_knot_vector[ii+1];//uj+1
		b = Du_span_knot_vector[ii];//u'j
		b1 = Du_span_knot_vector[ii+1];//u'j+1

		lj1 = nurbs_lenght_tot(ii, span_knot_vector, lenght_vector);//lj+1
		lj = nurbs_lenght_tot(ii-1,span_knot_vector, lenght_vector);//lj
		ll= pow((lj1-lj), 2);// (lj+1 -lj)^2
		c = (((a1-a)-(b*(lj1-lj)))/ ll);
		d = ((b1 + b)/ll) - (2*(a1-a)/ (pow((lj1-lj), 3)));

		nurbs_costant.push_back(a);
		nurbs_costant.push_back(b);
		nurbs_costant.push_back(c);
		nurbs_costant.push_back(d);
		nurbs_costant.push_back(lj);
		nurbs_costant.push_back(lj1);
		}
	return nurbs_costant;
	}

//Funzione inversa u(l)
double nurbs_uj_l(double l, std::vector<double> span_knot_vector, std::vector<double> lenght_vector, std::vector<double> nurbs_costant)
	{
	double a,b,c,d,lj1,lj,u_l=0,at2=0,at=0;
	int j=0;
	if(l == 0)
		{
		u_l=0;
		}
	else
		{
		for(unsigned ii=0; ii < nurbs_costant.size() - 5 ; ii=ii+6)
			{
			if((l <= lenght_vector[0]) && (at2 == 0) && (l>0))
				{
				at2+=1;
				a = nurbs_costant[0];
				b = nurbs_costant[1];
				c = nurbs_costant[2];
				d = nurbs_costant[3];
				lj = nurbs_costant[4];
				lj1 = nurbs_costant[5];

				u_l = a + b*(l)+ (c*pow(l,2))+ (d*pow(l,2)*(l-lj1));
				break;
				}
			else
				if((nurbs_lenght_tot(j, span_knot_vector,lenght_vector)> l) && (at == 0) && (u_l == 0))
					{
					at+=1;
					a = nurbs_costant[ii]; //uj
					b = nurbs_costant[ii+1];//u'j
					c = nurbs_costant[ii+2];
					d = nurbs_costant[ii+3];
					lj = nurbs_costant[ii+4];
					lj1 = nurbs_costant[ii+5];
					u_l = a + (b*(l-lj))+ (c*pow((l-lj),2))+ (d*pow((l-lj),2)*(l-lj1));
					break;
					}
			j++;
			}
		}
	return u_l;
	}




