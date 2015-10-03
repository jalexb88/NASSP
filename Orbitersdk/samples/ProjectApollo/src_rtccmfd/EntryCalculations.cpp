#include "EntryCalculations.h"

Entry::Entry(VESSEL *v, OBJHANDLE gravref, double GETbase, double EntryTIG, double EntryAng, double EntryLng, int critical, double entryrange, bool entrynominal, bool entrylongmanual)
{
	MA1 = 8e8;
	C0 = 1.81000432e8;
	C1 = 1.5078514;
	C2 = -6.49993054e-9;
	C3 = 9.769389245e-18;
	D2 = -4.8760771e-4;
	D3 = 4.5419476e-8;
	D4 = -1.4317675e-12;
	k1 = 7.0e6;
	k2 = 6.495e6;
	k3 = -0.06105;
	k4 = -0.10453;

	this->entrylongmanual = entrylongmanual;

	this->vessel = v;
	this->GETbase = GETbase;
	this->gravref = gravref;
	this->EntryAng = EntryAng;

	if (entrylongmanual)
	{
		this->EntryLng = EntryLng;
	}
	else
	{
		landingzone = (int)EntryLng;
		this->EntryLng = landingzonelong(landingzone, 0);
	}

	this->critical = critical;

	VECTOR3 R0, V0;
	double EntryInterface;

	vessel->GetRelativePos(gravref, R0);
	vessel->GetRelativeVel(gravref, V0);
	mjd = oapiGetSimMJD();

	Rot = OrbMech::J2000EclToBRCS(40222.525);

	R0B = mul(Rot, _V(R0.x, R0.z, R0.y));
	V0B = mul(Rot, _V(V0.x, V0.z, V0.y));

	get = (mjd - GETbase)*24.0*3600.0;

	EntryInterface = 400000.0 * 0.3048;

	hEarth = oapiGetObjectByName("Earth");

	RCON = oapiGetSize(oapiGetObjectByName("Earth")) + EntryInterface;
	RD = RCON;
	mu = GGRAV*oapiGetMass(hEarth);

	EntryTIGcor = EntryTIG;

	Tguess = PI2 / sqrt(mu)*OrbMech::power(length(R0), 1.5);

	tigslip = 100.0;
	ii = 0;
	entryphase = 0;

	bool stop;

	dt0 = EntryTIGcor - get;

	coast = new CoastIntegrator(R0B, V0B, mjd, dt0, gravref, hEarth);
	stop = false;
	while (stop == false)
	{
		stop = coast->iteration();
	}
	R11B = coast->R2;
	V11B = coast->V2;
	delete coast;

	x2 = OrbMech::cot(PI05 - EntryAng);
	if (length(R11B) > k1)
	{
		D1 = 1.6595;
	}
	else
	{
		D1 = 1.69107;
	}

	if (critical == 0)
	{
		EMSAlt = 284643.0*0.3048;
	}
	else
	{
		EMSAlt = 297431.0*0.3048;
	}
	revcor = -5;

	this->entryrange = entryrange;

	if (entryrange == 0)
	{
		rangeiter = 1;
	}
	else
	{
		rangeiter = 2;
	}

	R_E = oapiGetSize(gravref);
	earthorbitangle = (-31.7 - 2.15)*RAD;

	if (critical == 0)
	{
		this->entrynominal = entrynominal;
	}
	else
	{
		this->entrynominal = false;
	}
	precision = true;
}

Entry::Entry(OBJHANDLE gravref, int critical)
{
	this->critical = critical;
	this->gravref = gravref;

	double EntryInterface;
	EntryInterface = 400000.0 * 0.3048;

	hEarth = oapiGetObjectByName("Earth");
	mu = GGRAV*oapiGetMass(hEarth);

	RCON = oapiGetSize(hEarth) + EntryInterface;
	Rot = OrbMech::J2000EclToBRCS(40222.525);

	if (critical == 0)
	{
		EMSAlt = 284643.0*0.3048;
	}
	else
	{
		EMSAlt = 297431.0*0.3048;
	}
}

void Entry::newxt2(int n1, double xt2err, double &xt2_apo, double &xt2, double &xt2err_apo)
{
	double Dxt2;

	if (n1 == 1)
	{
		Dxt2 = xt2err;
	}
	else
	{
		Dxt2 = xt2err;//*((xt2_apo - xt2) / (xt2err - xt2err_apo));
	}
	xt2err_apo = xt2err;
	xt2_apo = xt2;
	xt2 = xt2 + Dxt2;
}

void Entry::xdviterator2(int f1, VECTOR3 R1B, VECTOR3 V1B, double theta1, double theta2, double theta3, VECTOR3 U_R1, VECTOR3 U_H, double dx, double xmin, double xmax, double &x)
{
	double xdes, xact, x_err,x_apo, x_err_apo, epsilon, p_CON, beta9;
	VECTOR3 Entry_DV, DV, V2;
	int i;
	i = 0;
	epsilon = pow(2.0, -20.0);

	xdes = tan(earthorbitangle - acos(R_E / length(R1B)));

	x_err_apo = 1000;

	while (abs(dx) > epsilon && i < 135)
	{
		dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);

		Entry_DV = _V(dotp(DV, U_H), 0.0, -dotp(DV, U_R1));

		xact = Entry_DV.z / Entry_DV.x;
		x_err = xdes - xact;

		//if (i == 0)
		//{
		//	dx = -0.01*(xdes - xact);
		//}
		//else
		//{
			//dx = x_err*((x_apo - x) / (x_err - x_err_apo));
		//}
		if (abs(x_err) > abs(x_err_apo))
		{
			dx = -dx*0.5;
		}
		beta9 = x + 1.1*dx;
		if (beta9 > xmax)
		{
			dx = (xmax - x) / 2.0;
		}
		else if (beta9 < xmin)
		{
			dx = (xmin - x) / 2.0;
		}
		x_err_apo = x_err;
		x_apo = x;
		x += dx;
		i++;
	}
}

void Entry::xdviterator(VECTOR3 R1B, VECTOR3 V1B, double theta1, double theta2, double theta3, VECTOR3 U_R1, VECTOR3 U_H, double dx, double xmin, double xmax, double &x)
{
	int i;
	double epsilon, p_CON, dvapo, beta8, beta9;
	VECTOR3 DV, V2;

	i = 0;
	epsilon = OrbMech::power(2.0, -20.0);

	dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);

	beta8 = xmax - xmin;
	if (beta8 <= dxmax)
	{
		dx = 0.5*beta8*OrbMech::sign(dx);
	}

	while (abs(dx) > epsilon && i < 135)
	{
		dvapo = length(DV);
		xapo = x;
		x += dx;
		dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
		if (length(DV) > dvapo)
		{
			dx = -dx*0.5;
		}
		beta9 = x + 1.1*dx;
		if (beta9 > xmax)
		{
			dx = (xmax - x) / 2.0;
		}
		else if (beta9 < xmin)
		{
			dx = (xmin - x) / 2.0;
		}
		i++;
	}
}

void Entry::dvcalc(VECTOR3 V1B, double theta1, double theta2, double theta3, double x, VECTOR3 U_R1, VECTOR3 U_H, VECTOR3 &V2, VECTOR3 &DV, double &p_CON)
{
	p_CON = theta2 / (theta1 - x*x);
	V2 = (U_R1*x + U_H)*theta3*sqrt(p_CON);
	DV = V2 - V1B;
}

void Entry::limitxchange(double theta1, double theta2, double theta3, VECTOR3 V1B, VECTOR3 U_R1, VECTOR3 U_H, double xmin, double xmax, double &x)
{
	double beta7, p_CON;
	VECTOR3 V2, DV;

	beta7 = abs(x - xapo);
	if (beta7>2.0*dxmax)
	{
		x = xapo;
		if (x > xmax)
		{
			x = xmax;
		}
		else
		{
			if (x < xmin)
			{
				x = xmin;
			}
		}
		dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
	}
}

void Entry::reentryconstraints(int n1, VECTOR3 R1B, VECTOR3 VEI)
{
	if (n1 == 0)
	{
		if (EntryAng == 0)
		{
			if (length(R1B) > k1)
			{
				x2 = k4;
			}
			else
			{
				x2 = k3;
			}
		}
		//n1 = 1;
	}
	else
	{
		if (EntryAng == 0)
		{
			double v2;
			v2 = length(VEI);
			x2_apo = D1 + D2*v2 + D3*v2*v2 + D4*v2*v2*v2;
		}
		else
		{
			x2_apo = x2;
		}
	}
}

void Entry::coniciter(VECTOR3 R1B, VECTOR3 V1B, double t1, double &theta_long, double &theta_lat, VECTOR3 &V2, double &x, double &dx, double &t21)
{
	VECTOR3 U_R1, U_H, REI, VEI;
	double MA2, x2_err, x2_err_apo, C_FPA;
	int n1;

	x2_err = 1.0;
	precomputations(1, R1B, V1B, U_R1, U_H, MA2, C_FPA);
	n1 = 1;
	while (abs(x2_err) > 0.0001 && n1 <= 10)
	{
		conicreturn(0, R1B, V1B, MA2, C_FPA, U_R1, U_H, V2, x, n1);
		t21 = OrbMech::time_radius(R1B, V2, RCON, -1, mu);
		OrbMech::rv_from_r0v0(R1B, V2, t21, REI, VEI, mu);
		reentryconstraints(n1, R1B, VEI);
		x2_err = x2_apo - x2;
		newxt2(n1, x2_err, x2_apo, x2, x2_err_apo);
		n1++;
	}
	t2 = t1 + t21;
	OrbMech::rv_from_r0v0(R1B, V2, t21, REI, VEI, mu);
	landingsite(REI, VEI, t2, theta_long, theta_lat);
}

void Entry::precisioniter(VECTOR3 R1B, VECTOR3 V1B, double t1, double &t21, double &x, double &theta_long, double &theta_lat, VECTOR3 &V2)
{
	double RD, R_ERR, dRCON, rPRE_apo, r1b, lambda, beta1, beta5, theta1, theta2, p_CON, C_FPA, MA2;
	VECTOR3 U_R1, U_V1, RPRE, VPRE, U_H, eta;
	int n1;

	n1 = 0;
	RCON = oapiGetSize(oapiGetObjectByName("Earth")) + 400000.0 * 0.3048;
	RD = RCON;
	R_ERR = 1000.0;

	U_R1 = unit(R1B);
	U_V1 = unit(V1B);
	C_FPA = dotp(U_R1, U_V1);
	if (abs(C_FPA) < 0.99966)
	{
		eta = crossp(R1B, V1B);
	}
	else
	{
		eta = _V(0.0, 0.0, 1.0);
	}
	if (eta.z < 0)
	{
		eta = -eta;
	}
	U_H = unit(crossp(eta, R1B));
	r1b = length(R1B);
	MA2 = C0 + C1*r1b + C2*r1b*r1b + C3*r1b*r1b*r1b;

	lambda = r1b / RCON;
	beta1 = 1.0 + x2*x2;
	beta5 = lambda*beta1;
	theta1 = beta5*lambda - 1.0;
	theta2 = 2.0*r1b*(lambda - 1.0);
	p_CON = theta2 / (theta1 - x*x);

	if (RCON - p_CON*beta1 >= 0)
	{
		phi2 = 1.0;
	}
	else
	{
		phi2 = -1.0;
	}

	finalstatevector(R1B, V2, beta1, t21, RPRE, VPRE);
	reentryconstraints(n1 + 1, R1B, VPRE);
	x2 = x2_apo;
	beta1 = 1.0 + x2*x2;
	R_ERR = length(RPRE) - RD;

	while ((abs(R_ERR) > 50.0 && n1 <= 10) || n1 == 0)
	{
		newrcon(n1, RD, length(RPRE), R_ERR, dRCON, rPRE_apo);
		conicreturn(1, R1B, V1B, MA2, C_FPA, U_R1, U_H, V2, x, n1);
		finalstatevector(R1B, V2, beta1, t21, RPRE, VPRE);
		//reentryconstraints(n1+1, R1B, VPRE);
		//x2 = x2_apo;
		//beta1 = 1.0 + x2*x2;
		R_ERR = length(RPRE) - RD;
		n1++;
	}
	t2 = t1 + t21;
	landingsite(RPRE, VPRE, t2, theta_long, theta_lat);
}

void Entry::newrcon(int n1, double RD, double rPRE, double R_ERR, double &dRCON, double &rPRE_apo)
{
	double S;

	if (n1 == 0)
	{
		this->RCON = RD*RD / rPRE;
		dRCON = RCON - RD;
	}
	else
	{
		S = dRCON / (rPRE_apo - rPRE);
		if (abs(S + 2.0) > 2.0)
		{
			S = -4.0;
		}
		dRCON = S*R_ERR;
		this->RCON = RCON + dRCON;
	}
	rPRE_apo = rPRE;
}

void Entry::finalstatevector(VECTOR3 R1B, VECTOR3 V2, double beta1, double &t21, VECTOR3 &RPRE, VECTOR3 &VPRE)
{
	VECTOR3 N;
	double beta12, x2PRE, c3, alpha_N, sing, cosg, p_N, beta2, beta3, beta4, RF, phi4, dt21, beta13, dt21apo, beta14;

	OrbMech::oneclickcoast(R1B, V2, mjd + (dt0 + dt1) / 24.0 / 3600.0, t21, RPRE, VPRE, hEarth, hEarth);

	beta12 = 100.0;
	x2PRE = 1000000;
	dt21apo = 100000000.0;

	while (abs(beta12) > 0.000007 && abs(x2 - x2PRE) > 0.00001)
	{
		c3 = length(RPRE)*pow(length(VPRE), 2.0) / mu;
		alpha_N = 2.0 - c3;
		N = crossp(unit(RPRE), unit(VPRE));
		sing = length(N);
		cosg = dotp(unit(RPRE), unit(VPRE));
		x2PRE = cosg / sing;
		p_N = c3*sing*sing;
		beta2 = p_N*beta1;
		beta3 = 1.0 - alpha_N*beta2;
		if (beta3 < 0)
		{
			beta4 = 1.0 / alpha_N;
		}
		else
		{
			beta4 = beta2 / (1.0 - phi2*sqrt(beta3));
		}
		beta12 = beta4 - 1.0;
		if (abs(beta12) > 0.000007)
		{
			RF = beta4*length(RPRE);
			if (beta12 > 0)
			{
				phi4 = -1.0;
			}
			else if (x2PRE > 0)
			{
				phi4 = -1.0;
			}
			else
			{
				phi4 = 1.0;
			}
			dt21 = OrbMech::time_radius(RPRE, VPRE*phi4, RF, -phi4, mu);
			dt21 = phi4*dt21;
			beta13 = dt21 / dt21apo;
			if (beta13 > 0)
			{
				beta14 = 1.0;
			}
			else
			{
				beta14 = -0.6;
			}
			if (beta13 / beta14 > 1.0)
			{
				dt21 = beta14*dt21apo;
			}
			dt21apo = dt21;
			OrbMech::oneclickcoast(RPRE, VPRE, mjd + (dt0 + dt1 + t21) / 24.0 / 3600.0, dt21, RPRE, VPRE, hEarth, hEarth);
			t21 += dt21;
		}
	}
}

void Entry::landingsite(VECTOR3 REI, VECTOR3 VEI, double t2, double &lambda, double &phi)
{
	double t32, v3, S_FPA, gammaE, phie, te, t_LS, Sphie, Cphie, tLSMJD, l, m, n;
	VECTOR3 R3, V3, UR3, U_H3, U_LS, LSEF;
	MATRIX3 R;

	t32 = OrbMech::time_radius(REI, VEI, length(REI) - 30480.0, -1, mu);
	OrbMech::rv_from_r0v0(REI, VEI, t32, R3, V3, mu);
	UR3 = unit(R3);
	v3 = length(V3);
	S_FPA = dotp(UR3, V3) / v3;
	gammaE = asin(S_FPA);
	augekugel(v3, gammaE, phie, te);


	t_LS = t2 + t32 + te;
	Sphie = sin(0.00029088821*phie);
	Cphie = cos(0.00029088821*phie);
	U_H3 = unit(crossp(crossp(R3, V3), R3));
	U_LS = UR3*Cphie + U_H3*Sphie;

	tLSMJD = GETbase + t_LS / 24.0 / 3600.0;
	U_LS = tmul(Rot, U_LS);
	U_LS = _V(U_LS.x, U_LS.z, U_LS.y);
	R = OrbMech::GetRotationMatrix2(hEarth, tLSMJD);
	LSEF = tmul(R, U_LS);
	l = LSEF.x;
	m = LSEF.z;
	n = LSEF.y;
	phi = asin(n);
	if (m > 0)
	{
		lambda = acos(l / cos(phi));
	}
	else
	{
		lambda = PI2 - acos(l / cos(phi));
	}
	if (lambda > PI) { lambda -= PI2; }
}

void Entry::conicreturn(int f1, VECTOR3 R1B, VECTOR3 V1B, double MA2, double C_FPA, VECTOR3 U_R1, VECTOR3 U_H, VECTOR3 &V2, double &x, int &n1)
{
	double theta1, theta2, theta3, xmin, xmax, p_CON, beta6, dx;
	VECTOR3 DV;
	conicinit(R1B, MA2, xmin, xmax, theta1, theta2, theta3);
	if (f1 == 0)
	{
		if (critical == 0)
		{
			if (C_FPA >= 0)
			{
				x = xmax;
				dx = -dxmax;
			}
			else
			{
				x = xmin;
				dx = dxmax;
			}
		}
		else if (ii == 0 && entryphase == 0)
		{
			x = xmin;
			dx = dxmax;
		}
		if (ii == 0)
		{
			if (critical == 0)
			{
				if (entrynominal)
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
					xdviterator2(f1, R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
				else
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
			}
			else if (critical == 1 && entryphase == 0)
			{
				xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
			}
			//else
			//{
			//	dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
			//}
		}
		else
		{
			if (critical == 0)
			{
				if (entrynominal)
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
					xdviterator2(f1, R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
				else
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
			}
			//else
			//{
			//	dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
			//}
		}
	}
	else
	{
		double r1b, lambda, beta1, beta5;
		r1b = length(R1B);
		lambda = r1b / RCON;
		beta1 = 1.0 + x2*x2;
		beta5 = lambda*beta1;
		beta6 = beta5*(2.0 - lambda) - 1.0;
		if (critical == 0)
		{
			if (beta6 > 0)
			{
				xlim = sqrt(beta6);
				if (phi2 == 1)
				{
					xmax = xlim;
					xmin = -xlim;
					if (x > 0)
					{
						x = xmax;
						dx = -dxmax;
					}
					else
					{
						x = xmin;
						dx = dxmax;
					}
				}
				else
				{
					if (x > 0)
					{
						xmin = xlim;
						x = xmax;
						dx = -dxmax;
					}
					else
					{

						xmax = -xlim;
						x = xmin;
						dx = dxmax;
					}
				}
			}
			else
			{
				if (phi2 == 1)
				{
					phi2 = -1.0;
					n1 = -1;
				}
				if (x > 0)
				{
					x = xmax;
					dx = -dxmax;
				}
				else
				{
					x = xmin;
					dx = dxmax;
				}
			}

			if (entrynominal)
			{
				xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				//conicinit(R1B, MA2, xmin, xmax, theta1, theta2, theta3);
				xdviterator2(f1, R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
			}
			else
			{
				xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
			}
		}
		//else
		//{
		//	dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
		//}
	}
	dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
}

void Entry::conicinit(VECTOR3 R1B, double MA2, double &xmin, double &xmax, double &theta1, double &theta2, double &theta3)
{
	double r1b, lambda, beta1, beta5, beta10;
	r1b = length(R1B);

	lambda = r1b / RCON;
	beta1 = 1.0 + x2*x2;
	beta5 = lambda*beta1;
	theta1 = beta5*lambda - 1.0;
	theta2 = 2.0*r1b*(lambda - 1.0);
	theta3 = sqrt(mu) / r1b;
	beta10 = beta5*(MA1 - r1b) / (MA1 - RCON) - 1.0;
	if (beta10 < 0.0)
	{
		xmin = 0.0;
	}
	else
	{
		xmin = -sqrt(beta10);
	}
	dxmax = -xmin / 16.0;
	beta10 = beta5*(MA2 - r1b) / (MA2 - RCON) - 1.0;
	if (beta10 < 0.0)
	{
		xmax = 0.0;
	}
	else
	{
		xmax = sqrt(beta10);
	}
}

void Entry::precomputations(bool x2set, VECTOR3 R1B, VECTOR3 V1B, VECTOR3 &U_R1, VECTOR3 &U_H, double &MA2, double &C_FPA)
{
	VECTOR3 U_V1, eta;
	double r1b;

	U_R1 = unit(R1B);
	U_V1 = unit(V1B);
	C_FPA = dotp(U_R1, U_V1);
	if (abs(C_FPA) < 0.99966)
	{
		eta = crossp(R1B, V1B);
	}
	else
	{
		eta = _V(0.0, 0.0, 1.0);
	}
	if (eta.z < 0)
	{
		eta = -eta;
	}
	U_H = unit(crossp(eta, R1B));
	r1b = length(R1B);
	MA2 = C0 + C1*r1b + C2*r1b*r1b + C3*r1b*r1b*r1b;
	if (x2set)
	{
		reentryconstraints(0, R1B, _V(0, 0, 0));
	}
}

bool Entry::EntryIter()
{
	double theta_long, theta_lat, dlng,t21;
	VECTOR3 R1B, V1B, V2;

	dt1 = EntryTIGcor - get - dt0;
	OrbMech::oneclickcoast(R11B, V11B, mjd + dt0 / 24.0 / 3600.0, dt1, R1B, V1B, hEarth, hEarth);
	coniciter(R1B, V1B, EntryTIGcor, theta_long, theta_lat, V2, x, dx, t21);
	if (entryphase == 1 && precision)
	{
		precisioniter(R1B, V1B, EntryTIGcor, t21, x, theta_long, theta_lat, V2);
	}

	if (!entrylongmanual)
	{
		EntryLng = landingzonelong(landingzone, theta_lat);
	}

	dlng = EntryLng - theta_long;
	if (abs(dlng) > PI)
	{
		dlng = dlng - OrbMech::sign(dlng)*PI2;
	}
	if (critical == 0)
	{
		tigslip = Tguess*dlng / PI2;
		EntryTIGcor += tigslip;
	}
		else
		{
			if (ii == 0 && entryphase == 0)
			{
				dx = -dlng * RAD;
				xapo = x;
				dlngapo = theta_long;
				x += dx;

			}
			else
			{
				dx = (x - xapo) / (theta_long - dlngapo)*dlng;
				if (length(V2-V1B) > 2804.0)
				{
					dx = 0.5*max(1.0, revcor);
					revcor++;
				}
				else if (abs(dx) > dxmax)
				{
					dx = OrbMech::sign(dx)*dxmax;
				}
				xapo = x;
				dlngapo = theta_long;
				x += dx;

			}
		}


	ii++;

	if ((abs(dlng) > 0.005*RAD && ii < 40) || entryphase == 0)
	{
		if (abs(tigslip) < 10.0 || (critical > 0 && abs(dlng)<0.1*RAD))
		{
			if (entryphase == 0)
			{
				entryphase = 1;
				ii = 0;
			}
		}
		return false;
	}
	else
	{
		if (precision && entrynominal)
		{
			if (abs(x - xlim) < OrbMech::power(2.0, -20.0)*2.0 || abs(x + xlim) < OrbMech::power(2.0, -20.0)*2.0 || ii == 40)
			{
				ii = 2;
				precision = false;
				return false;
			}
		}
		VECTOR3 R05G, V05G, REI, VEI, R3, V3, UR3, U_R1, U_H, DV;
		double t32, dt22, v3, S_FPA, gammaE, phie, te, MA2, C_FPA;

		precomputations(0, R1B, V1B, U_R1, U_H, MA2, C_FPA);

		t2 = EntryTIGcor + t21;
		OrbMech::time_radius_integ(R1B, V2, mjd + (dt0 + dt1) / 24.0 / 3600.0, RD, -1, hEarth, hEarth, REI, VEI);//Maneuver to Entry Interface (400k ft)

		t32 = OrbMech::time_radius(REI, VEI, length(REI) - 30480.0, -1, mu);
		OrbMech::rv_from_r0v0(REI, VEI, t32, R3, V3, mu); //Entry Interface to 300k ft

		dt22 = OrbMech::time_radius(R3, V3, length(R3) - (300000.0 * 0.3048 - EMSAlt), -1, mu);
		OrbMech::rv_from_r0v0(R3, V3, dt22, R05G, V05G, mu); //300k ft to 0.05g

		UR3 = unit(R3);
		v3 = length(V3);
		S_FPA = dotp(UR3, V3) / v3;
		gammaE = asin(S_FPA);
		augekugel(v3, gammaE, phie, te);

		DV = V2 - V1B;
		Entry_DV = _V(dotp(DV, U_H), 0.0, -dotp(DV, U_R1));
		EntryRTGO = phie - 3437.7468*acos(dotp(unit(R3), unit(R05G)));
		EntryVIO = length(V05G);
		EntryRET = t2 - EntryTIGcor + t32 + dt22;
		EntryAng = atan(x2);//asin(dotp(unit(REI), VEI) / length(VEI));

		EntryLngcor = theta_long;
		EntryLatcor = theta_lat;
		return true;
	}
}

/*bool Entry::EntryIter2()
{
	double dt1, r1b, MA2, lambda, beta1, beta5, beta10;
	double dt2, R_ERR, dt21apo, beta12, x2PRE, alpha_N, c3, sing, cosg, p_N, beta2, beta3, beta4, RF, phi4;
	double dt21, beta13, beta14, dRCON, S, RPRE_apo, t32, v3, S_FPA, gammaE, phie, te, t_LS, Sphie, Cphie, tLSMJD, l, m, n, phi, lambda2;
	double dlng, beta6, xlim, C_FPA;
	bool stop;
	VECTOR3 U_V1, eta, RPRE, VPRE, N, REI, R3, V3, UR3, U_H3, U_LS, LSEF, U_R1, U_H, DV, R1B, V1B, V2, VEI;
	MATRIX3 R;
	double x2_err,x2_err_apo;
	double theta1, theta2, theta3, x, dv, p_CON;
	double xmin, xmax, dx;
	int n1;

	x2_err = 1.0;

	RCON = RD;
	dt1 = EntryTIGcor - get-dt0;

	if (dt1 != 0)
	{
		coast = new CoastIntegrator(R11B, V11B, mjd + dt0/24.0/3600.0, dt1, hEarth, hEarth);
		stop = false;
		while (stop == false)
		{
			stop = coast->iteration();
		}
		R1B = coast->R2;
		V1B = coast->V2;
		delete coast;
	}
	else
	{
		R1B = R11B;
		V1B = V11B;
	}

	

	//x[0] = 0.0;
	//x[1] = 0.0;
	//dv[0] = 100000.0;
	//V2 = entrydviter(R1B, V1B, RCON, x[1], x2, mu);
	//dv[1] = length(V2);

	//h = 0.00001;

	U_R1 = unit(R1B);
	U_V1 = unit(V1B);
	C_FPA = dotp(U_R1, U_V1);
	if (abs(C_FPA) < 0.99966)
	{
		eta = crossp(R1B, V1B);
	}
	else
	{
		eta = _V(0.0, 0.0, 1.0);
	}
	if (eta.z < 0)
	{
		eta = -eta;
	}
	U_H = unit(crossp(eta, R1B));
	r1b = length(R1B);
	MA2 = C0 + C1*r1b + C2*r1b*r1b + C3*r1b*r1b*r1b;
	n1 = 0;
	reentryconstraints(n1, R1B, _V(0,0,0));
	f2 = 0;
	while (abs(x2_err)>0.0001)
	{
		lambda = r1b / RCON;
		beta1 = 1.0 + x2*x2;
		beta5 = lambda*beta1;
		theta1 = beta5*lambda - 1.0;
		theta2 = 2.0*r1b*(lambda - 1.0);
		theta3 = sqrt(mu) / r1b;
		beta10 = beta5*(MA1 - r1b) / (MA1 - RCON) - 1.0;
		if (beta10 < 0.0)
		{
			xmin = 0.0;
		}
		else
		{
			xmin = -sqrt(beta10);
		}
		dxmax = -xmin / 16.0;
		beta10 = beta5*(MA2 - r1b) / (MA2 - RCON) - 1.0;
		if (beta10 < 0.0)
		{
			xmax = 0.0;
		}
		else
		{
			xmax = sqrt(beta10);
		}
		if (critical == 0)
		{
			if (C_FPA >= 0)
			{
				x = xmax;
				dx = -dxmax;
			}
			else
			{
				x = xmin;
				dx = dxmax;
			}
		}
		else if (ii == 0)
		{
			x = xmin;
			dx = dxmax;
		}
		if (ii == 0)
		{
			if (critical == 0 || critical == 1)
			{
				if (entrynominal)
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
					xdviterator2(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, xmin, xmax, x);
				}
				else
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
			}
			else
			{
				dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
			}
		}
		else
		{
			if (critical == 0)
			{
				if (entrynominal)
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
					xdviterator2(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, xmin, xmax, x);
				}
				else
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
			}
			else
			{
				dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
			}
		}

		//Entry_DV = _V(dotp(DV, U_H), 0.0, -dotp(DV, U_R1));

		dt2 = OrbMech::time_radius(R1B, V2, RCON, -1, mu);
		OrbMech::rv_from_r0v0(R1B, V2, dt2, REI, VEI, mu);
		double rtest;
		rtest = length(REI);
		reentryconstraints(n1, R1B, VEI);
		x2_err = x2_apo - x2;

		newxt2(n1, x2_err, x2_apo, x2, x2_err_apo);
		n1++;
	}

	if (entryphase == 0)
	{
		OrbMech::rv_from_r0v0(R1B, V2, dt2, REI, VEI, mu);
		t32 = OrbMech::time_radius(REI, VEI, RCON - 30480.0, -1, mu);
		OrbMech::rv_from_r0v0(REI, VEI, t32, R3, V3, mu);
		UR3 = unit(R3);
		v3 = length(V3);
		S_FPA = dotp(UR3, V3) / v3;
		gammaE = asin(S_FPA);
		augekugel(v3, gammaE, phie, te);

		t2 = EntryTIGcor + dt2;
		t_LS = t2 + t32 + te;
		Sphie = sin(0.00029088821*phie);
		Cphie = cos(0.00029088821*phie);
		U_H3 = unit(crossp(crossp(R3, V3), R3));
		U_LS = UR3*Cphie + U_H3*Sphie;

		tLSMJD = GETbase + t_LS / 24.0 / 3600.0;
		U_LS = tmul(Rot, U_LS);
		U_LS = _V(U_LS.x, U_LS.z, U_LS.y);
		R = OrbMech::GetRotationMatrix2(hEarth, tLSMJD);
		LSEF = tmul(R, U_LS);
		l = LSEF.x;
		m = LSEF.z;
		n = LSEF.y;
		phi = asin(n);
		if (m > 0)
		{
			lambda2 = acos(l / cos(phi));
		}
		else
		{
			lambda2 = PI2 - acos(l / cos(phi));
		}
		if (lambda2 > PI){ lambda2 -= PI2; }

		dlng = EntryLng - lambda2;
		if (abs(dlng) > PI)
		{
			dlng = dlng - OrbMech::sign(dlng)*PI2;
		}
		EntryLngcor = lambda2;
		EntryLatcor = phi;

		if (critical == 0)
		{
			tigslip = Tguess*dlng / PI2;
			EntryTIGcor += tigslip;
		}
		else
		{
			if (ii == 0)
			{
				dx = -dlng * RAD;
				xapo = x;
				dlngapo = lambda2;
				x += dx;

			}
			else
			{
				dx = (x - xapo) / (lambda2 - dlngapo)*dlng;
				if (dv > 2804.0)
				{
					dx = 0.5*max(1.0,revcor);
					revcor++;
				}
				else if (abs(dx) > dxmax)
				{
					dx = OrbMech::sign(dx)*dxmax;
				}
				xapo = x;
				dlngapo = lambda2;
				x += dx;

			}
		}
	}
	else
	{
		if (RCON - p_CON*beta1 >= 0)
		{
			phi2 = 1.0;
		}
		else
		{
			phi2 = -1.0;
		}

		//phi2 = 1.0;
		R_ERR = 1000000.0;

		n1 = 0;
		//RD = RCON;

		while (abs(R_ERR) > 100.0 || n1 == 0)
		{
			dt21apo = 100000000.0;

			coast = new CoastIntegrator(R1B, V2, mjd + (dt0 + dt1) / 24.0 / 3600.0, dt2, hEarth, hEarth);
			stop = false;
			while (stop == false)
			{
				stop = coast->iteration();
			}
			RPRE = coast->R2;
			VPRE = coast->V2;
			delete coast;

			beta12 = 100.0;
			x2PRE = 1000000;
			while (abs(beta12) > 0.000007 && abs(x2 - x2PRE) > 0.00001)
			{
				//gamma2 = asin((dotp(unit(RPRE), unit(VPRE))));
				// cot(PI05 - gamma2);
				c3 = length(RPRE)*pow(length(VPRE), 2.0) / mu;
				alpha_N = 2.0 - c3;
				N = crossp(unit(RPRE), unit(VPRE));
				sing = length(N);
				cosg = dotp(unit(RPRE), unit(VPRE));
				x2PRE = cosg / sing;
				p_N = c3*sing*sing;
				beta2 = p_N*beta1;
				/*beta2 = 1.0 - alpha_N*p_N*beta1;// p_N*beta1;
				if (beta2 > 0)
				{
				beta3 = sqrt(beta2);
				}
				else
				{
				beta3 = 0;
				}
				//beta4 = RD / length(RPRE);// (1 + beta3*phi2) / alpha_N;
				beta3 = 1.0 - alpha_N*beta2;
				if (beta3 < 0)
				{
					beta4 = 1.0 / alpha_N;
				}
				else
				{
					beta4 = beta2 / (1.0 - phi2*sqrt(beta3));
				}
				beta12 = beta4 - 1.0;
				if (abs(beta12) > 0.000007)
				{
					RF = beta4*length(RPRE);
					if (beta12 > 0)
					{
						phi4 = -1.0;
					}
					else if (x2PRE > 0)
					{
						phi4 = -1.0;
					}
					else
					{
						phi4 = 1.0;
					}
					dt21 = OrbMech::time_radius(RPRE, VPRE*phi4, RF, -phi4, mu);
					dt21 = phi4*dt21;
					beta13 = dt21 / dt21apo;
					if (beta13 > 0)
					{
						beta14 = 1.0;
					}
					else
					{
						beta14 = -0.6;
					}
					if (beta13 / beta14 > 1.0)
					{
						dt21 = beta14*dt21apo;
					}
					dt21apo = dt21;
					coast = new CoastIntegrator(RPRE, VPRE, mjd + (dt0 + dt1 + dt2) / 24.0 / 3600.0, dt21, hEarth, hEarth);
					dt2 += dt21;
					stop = false;
					while (stop == false)
					{
						stop = coast->iteration();
					}
					RPRE = coast->R2;
					VPRE = coast->V2;
					delete coast;
				}
			}

			R_ERR = length(RPRE) - RD;
			if (n1 == 0)
			{
				RCON = RD*RD / length(RPRE);
				dRCON = RCON - RD;
			}
			else
			{
				S = dRCON / (RPRE_apo - length(RPRE));
				if (abs(S + 2.0) > 2.0)
				{
					S = -4.0;
				}
				dRCON = S*R_ERR;
				RCON = RCON + dRCON;
			}
			RPRE_apo = length(RPRE);

			n1++;
			f2 = 0;
			lambda = r1b / RCON;
			beta1 = 1.0 + x2*x2;
			beta5 = lambda*beta1;
			theta1 = beta5*lambda - 1.0;
			theta2 = 2.0*length(R1B)*(lambda - 1.0);
			theta3 = sqrt(mu) / length(R1B);
			beta10 = beta5*(MA1 - r1b) / (MA1 - RCON) - 1.0;
			if (beta10 < 0.0)
			{
				xmin = 0.0;
			}
			else
			{
				xmin = -sqrt(beta10);
			}
			dxmax = -xmin / 16.0;
			beta10 = beta5*(MA2 - r1b) / (MA2 - RCON) - 1.0;
			if (beta10 < 0.0)
			{
				xmax = 0.0;
			}
			else
			{
				xmax = sqrt(beta10);
			}
			//x_apo = x;
			beta6 = beta5*(2.0 - lambda) - 1.0;
			if (critical == 0)
			{
				if (beta6 > 0)
				{
					xlim = sqrt(beta6);
					if (phi2 == 1)
					{
						xmax = xlim;
						xmin = -xlim;
						if (x > 0)
						{
							x = xmax;
							dx = -dxmax;
						}
						else
						{
							x = xmin;
							dx = dxmax;
						}
					}
					else
					{
						if (x > 0)
						{
							xmin = xlim;
							x = xmax;
							dx = -dxmax;
						}
						else
						{
							xmax = -xlim;
							x = xmin;
							dx = dxmax;
						}
					}
				}
				else
				{
					if (phi2 == 1)
					{
						phi2 = -1.0;
						n1 = 0;
					}
					if (x > 0)
					{
						x = xmax;
						dx = -dxmax;
					}
					else
					{
						x = xmin;
						dx = dxmax;
					}
				}

				if (entrynominal)
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
					xdviterator2(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, xmin, xmax, x);
				}
				else
				{
					xdviterator(R1B, V1B, theta1, theta2, theta3, U_R1, U_H, dx, xmin, xmax, x);
				}
			}
			else
			{
				dvcalc(V1B, theta1, theta2, theta3, x, U_R1, U_H, V2, DV, p_CON);
			}
		}
		//Entry_DV = _V(dotp(DV, U_H), 0.0, -dotp(DV, U_R1));

		//dt2 = OrbMech::time_radius(R1B, V2, RCON, -1, mu);

		//rv_from_r0v0(R1B, V2, dt2, REI, VEI, mu);
		t32 = OrbMech::time_radius(RPRE, VPRE, length(RPRE) - 30480.0, -1, mu);
		OrbMech::rv_from_r0v0(RPRE, VPRE, t32, R3, V3, mu);
		UR3 = unit(R3);
		v3 = length(V3);
		S_FPA = dotp(UR3, V3) / v3;
		gammaE = asin(S_FPA);
		augekugel(v3, gammaE, phie, te);

		t2 = EntryTIGcor + dt2;
		t_LS = t2 + t32 + te;
		Sphie = sin(0.00029088821*phie);
		Cphie = cos(0.00029088821*phie);
		U_H3 = unit(crossp(crossp(R3, V3), R3));
		U_LS = UR3*Cphie + U_H3*Sphie;

		tLSMJD = GETbase + t_LS / 24.0 / 3600.0;
		U_LS = tmul(Rot, U_LS);
		U_LS = _V(U_LS.x, U_LS.z, U_LS.y);
		R = OrbMech::GetRotationMatrix2(hEarth, tLSMJD);
		LSEF = tmul(R, U_LS);
		l = LSEF.x;
		m = LSEF.z;
		n = LSEF.y;
		phi = asin(n);
		if (m > 0)
		{
			lambda2 = acos(l / cos(phi));
		}
		else
		{
			lambda2 = PI2 - acos(l / cos(phi));
		}
		if (lambda2 > PI){ lambda2 -= PI2; }

		dlng = EntryLng - lambda2;
		if (abs(dlng) > PI)
		{
			dlng = dlng-OrbMech::sign(dlng)*PI2;
		}
		EntryLngcor = lambda2;
		EntryLatcor = phi;

		if (critical == 0)
		{
			tigslip = Tguess*dlng / PI2;
			EntryTIGcor += tigslip;
		}
		else
		{
			if (ii == 0)
			{
				dx = -dlng * RAD;
				xapo = x;
				dlngapo = lambda2;
				x += dx;

			}
			else
			{
				dx = (x - xapo) / (lambda2 - dlngapo)*dlng;
				if (dv > 2804.0)
				{
					dx = 0.5*max(1.0, revcor);
					revcor++;
				}
				else if (abs(dx) > dxmax)
				{
					dx = OrbMech::sign(dx)*dxmax;
				}
				xapo = x;
				dlngapo = lambda2;
				x += dx;
				
			}
		}
	}
	ii++;

	if ((abs(dlng) > 0.005*RAD && ii < 100) || entryphase == 0)
	{
		if (abs(tigslip) < 10.0 || (critical > 0 && abs(dlng)<0.1*RAD))
		{
			entryphase = 1;
		}
		return false;
	}
	else
	{
		VECTOR3 R05G, V05G;
		double dt22 = OrbMech::time_radius(R3, V3, length(R3) - (300000.0 * 0.3048 - EMSAlt), -1, mu);
		OrbMech::rv_from_r0v0(R3, V3, dt22, R05G, V05G, mu);

		Entry_DV = _V(dotp(DV, U_H), 0.0, -dotp(DV, U_R1));
		EntryRTGO = phie - 3437.7468*acos(dotp(unit(R3), unit(R05G)));
		EntryVIO = length(V05G);
		EntryRET = dt2 + t32 + dt22;
		EntryAng = atan(x2);
		return true;
	}
}*/

void Entry::EntryUpdateCalc()
{
	VECTOR3 R0, V0, R0B, V0B, REI, VEI, R3, V3, UR3, U_H3, U_LS, LSEF;
	double mjd, RCON, mu, dt2, t32, v3, S_FPA, gammaE, phie, te, t2, t_LS, Sphie, Cphie, tLSMJD, l, m, n, phi, lambda2, EntryInterface;
	MATRIX3 Rot, R;
	OBJHANDLE hEarth;
	VECTOR3 R05G, V05G;
	double dt22;

	hEarth = oapiGetObjectByName("Earth");

	EntryInterface = 400000 * 0.3048;
	RCON = oapiGetSize(hEarth) + EntryInterface;
	mu = GGRAV*oapiGetMass(gravref);

	vessel->GetRelativePos(gravref, R0);
	vessel->GetRelativeVel(gravref, V0);
	mjd = oapiGetSimMJD();

	Rot = OrbMech::J2000EclToBRCS(40222.525);

	R0B = mul(Rot, _V(R0.x, R0.z, R0.y));
	V0B = mul(Rot, _V(V0.x, V0.z, V0.y));

	dt2 = OrbMech::time_radius_integ(R0B, V0B, mjd, RCON, -1, gravref, hEarth, REI, VEI);

	//rv_from_r0v0(R0B, V0B, dt2, REI, VEI, mu);

	t32 = OrbMech::time_radius(REI, VEI, RCON - 30480.0, -1, mu);
	OrbMech::rv_from_r0v0(REI, VEI, t32, R3, V3, mu);
	UR3 = unit(R3);
	v3 = length(V3);
	S_FPA = dotp(UR3, V3) / v3;
	gammaE = asin(S_FPA);
	augekugel(v3, gammaE, phie, te);

	for (int iii = 0;iii < rangeiter;iii++)
	{
		t2 = dt2;
		t_LS = t2 + t32 + te;
		Sphie = sin(0.00029088821*phie);
		Cphie = cos(0.00029088821*phie);
		U_H3 = unit(crossp(crossp(R3, V3), R3));
		U_LS = UR3*Cphie + U_H3*Sphie;

		tLSMJD = mjd + t_LS / 24.0 / 3600.0;
		U_LS = tmul(Rot, U_LS);
		U_LS = _V(U_LS.x, U_LS.z, U_LS.y);
		R = OrbMech::GetRotationMatrix2(gravref, tLSMJD);
		LSEF = tmul(R, U_LS);
		l = LSEF.x;
		m = LSEF.z;
		n = LSEF.y;
		phi = asin(n);
		if (m > 0)
		{
			lambda2 = acos(l / cos(phi));
		}
		else
		{
			lambda2 = PI2 - acos(l / cos(phi));
		}
		if (lambda2 > PI) { lambda2 -= PI2; }

		EntryLatPred = phi;
		EntryLngPred = lambda2;

		
		dt22 = OrbMech::time_radius(R3, V3, length(R3) - (300000.0 * 0.3048 - EMSAlt), -1, mu);
		OrbMech::rv_from_r0v0(R3, V3, dt22, R05G, V05G, mu);

		EntryRTGO = phie - 3437.7468*acos(dotp(unit(R3), unit(R05G)));
		if (entryrange != 0)
		{
			te *= entryrange / EntryRTGO;
			phie = entryrange + 3437.7468*acos(dotp(unit(R3), unit(R05G)));
		}
	}
	EntryVIO = length(V05G);
	EntryRET = dt2 + t32 + dt22;
}

void Entry::Reentry(VECTOR3 REI, VECTOR3 VEI, double mjd0)
{
	double t32,v3,S_FPA, gammaE,phie,te,t_LS,Sphie,Cphie, tLSMJD,l,m,n,phi,lambda2;
	VECTOR3 R3, V3, UR3, U_H3, U_LS, LSEF;
	MATRIX3 R;

	t32 = OrbMech::time_radius(REI, VEI, RCON - 30480.0, -1, mu);
	OrbMech::rv_from_r0v0(REI, VEI, t32, R3, V3, mu);
	UR3 = unit(R3);
	v3 = length(V3);
	S_FPA = dotp(UR3, V3) / v3;
	gammaE = asin(S_FPA);
	augekugel(v3, gammaE, phie, te);

	t_LS = t32 + te;
	Sphie = sin(0.00029088821*phie);
	Cphie = cos(0.00029088821*phie);
	U_H3 = unit(crossp(crossp(R3, V3), R3));
	U_LS = UR3*Cphie + U_H3*Sphie;

	tLSMJD = mjd0 + t_LS / 24.0 / 3600.0;
	U_LS = tmul(Rot, U_LS);
	U_LS = _V(U_LS.x, U_LS.z, U_LS.y);
	R = OrbMech::GetRotationMatrix2(gravref, tLSMJD);
	LSEF = tmul(R, U_LS);
	l = LSEF.x;
	m = LSEF.z;
	n = LSEF.y;
	phi = asin(n);
	if (m > 0)
	{
		lambda2 = acos(l / cos(phi));
	}
	else
	{
		lambda2 = PI2 - acos(l / cos(phi));
	}
	if (lambda2 > PI){ lambda2 -= PI2; }

	EntryLatPred = phi;
	EntryLngPred = lambda2;

	VECTOR3 R05G, V05G;
	double dt22 = OrbMech::time_radius(R3, V3, length(R3) - (300000.0 * 0.3048 - EMSAlt), -1, mu);
	OrbMech::rv_from_r0v0(R3, V3, dt22, R05G, V05G, mu);

	EntryRTGO = phie - 3437.7468*acos(dotp(unit(R3), unit(R05G)));
	EntryVIO = length(V05G);
	EntryRET = t32 + dt22;
}

void Entry::augekugel(double ve, double gammae, double &phie, double &Te)
{
	double vefps, gammaedeg, K1, K2;

	vefps = ve / 0.3048;
	gammaedeg = gammae * DEG;
	if (vefps <= 21000)
	{
		K1 = 5500.0;
	}
	else
	{
		if (vefps <= 28000.0)
		{
			K1 = 2400.0 + 0.443*(28000.0 - vefps);
		}
		else
		{
			K1 = 2400.0;
		}
	}
	if (vefps <= 24000.0)
	{
		K2 = -3.2 - 0.001222*(24000.0 - vefps);
	}
	else
	{
		if (vefps <= 28400.0)
		{
			K2 = 1.0 - 0.00105*(28000.0 - vefps);
		}
		else
		{
			K2 = 2.4 + 0.000285*(vefps - 32000.0);
		}
	}
	phie = K1 / (abs(gammaedeg) - K2);
	if (vefps < 26000.0)
	{
		Te = 8660.0 * phie / vefps;
	}
	else
	{
		Te = phie / 3.0;
	}
}

double Entry::MPL(double lat) //Calculate the splashdown longitude from the latitude for the Mid-Pacific landing area
{
	return -165.0*RAD;
}

double Entry::EPL(double lat) //Calculate the splashdown longitude from the latitude for the East-Pacific landing area
{
	if (lat > 21.0*RAD)
	{
		return (-135.0*RAD + 122.0*RAD) / (40.0*RAD - 21.0*RAD)*(lat - 21.0*RAD) - 122.0*RAD;
	}
	else if (lat > -11.0*RAD)
	{
		return (-122.0*RAD + 89.0*RAD) / (21.0*RAD + 11.0*RAD)*(lat + 11.0*RAD) - 89.0*RAD;
	}
	else
	{
		return (-89.0*RAD + 83.0*RAD) / (-11.0*RAD + 40.0*RAD)*(lat + 40.0*RAD) - 83.0*RAD;
	}
}

double Entry::AOL(double lat)
{
	if (lat > 10.0*RAD)
	{
		return -30.0*RAD;
	}
	else if (lat > -5.0*RAD)
	{
		return (-30.0*RAD + 25.0*RAD) / (10.0*RAD + 5.0*RAD)*(lat + 5.0*RAD) - 25.0*RAD;
	}
	else
	{
		return -25.0*RAD;
	}
}

double Entry::IOL(double lat)
{
	return 65.0*RAD;
}

double Entry::WPL(double lat)
{
	if (lat > 10.0*RAD)
	{
		return 150.0*RAD;
	}
	else if (lat > -15.0*RAD)
	{
		return (150.0*RAD - 170.0*RAD) / (10.0*RAD + 15.0*RAD)*(lat + 15.0*RAD) + 170.0*RAD;
	}
	else
	{
		return 170.0*RAD;
	}
}

double Entry::landingzonelong(int zone, double lat)
{
	if (zone == 0)
	{
		return MPL(lat);
	}
	else if (zone == 1)
	{
		return EPL(lat);
	}
	else if (zone == 2)
	{
		return AOL(lat);
	}
	else if (zone == 3)
	{
		return IOL(lat);
	}
	else
	{
		return WPL(lat);
	}
}