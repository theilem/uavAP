////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * UTMToLatLong.h
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UTMTOLATLONG_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UTMTOLATLONG_H_

// LatLong-UTM.c++
// Conversions:  LatLong to UTM;  and UTM to LatLong;
// by Eugene Reimer, ereimer@shaw.ca, 2002-December;
// with LLtoUTM & UTMtoLL routines based on those by Chuck Gantz chuck.gantz@globalstar.com;
// with ellipsoid & datum constants from Peter H Dana website (http://www.colorado.edu/geography/gcraft/notes/datum/edlist.html);
//
// Usage:  see the Usage() routine below;
//
// Copyright © 1995,2002,2010 Eugene Reimer, Peter Dana, Chuck Gantz.  Released under the GPL;  see http://www.gnu.org/licenses/gpl.html
// (Peter Dana's non-commercial clause precludes using the LGPL)

#include <cmath>			//2010-08-11: was <math.h>
#include <cstdio>			//2010-08-11: was <stdio.h>
#include <cstdlib>			//2010-08-11: was <stdlib.h>
#include <cstring>			//2010-08-11: was <string.h>
#include <cctype>			//2010-08-11: was <ctype.h>
#include <iostream>			//2010-08-11: was <iostream.h>
#include <iomanip>			//2010-08-11: was <iomanip.h>
//2010-08-11: added
const double PI = 4 * atan(1);	//Gantz used: PI=3.14159265;
const double deg2rad = PI / 180;
const double rad2deg = 180 / PI;
const double k0 = 0.9996;

class Ellipsoid
{
public:
	Ellipsoid(int id, const std::string& name, double radius, double fr)
	{
		Name = name;
		EquatorialRadius = radius;
		eccSquared = 2 / fr - 1 / (fr * fr);
	}
	std::string Name;
	double EquatorialRadius;
	double eccSquared;
};

static Ellipsoid ellip[] =
{		//converted from PeterDana website, by Eugene Reimer 2002dec
//		 eId,  Name,		   EquatorialRadius,    1/flattening;
		Ellipsoid(0, "Airy1830", 6377563.396, 299.3249646), Ellipsoid(1, "AiryModified",
				6377340.189, 299.3249646), Ellipsoid(2, "AustralianNational", 6378160, 298.25),
				Ellipsoid(3, "Bessel1841Namibia", 6377483.865, 299.1528128), Ellipsoid(4,
						"Bessel1841", 6377397.155, 299.1528128), Ellipsoid(5, "Clarke1866",
						6378206.4, 294.9786982), Ellipsoid(6, "Clarke1880", 6378249.145, 293.465),
				Ellipsoid(7, "EverestIndia1830", 6377276.345, 300.8017), Ellipsoid(8,
						"EverestSabahSarawak", 6377298.556, 300.8017), Ellipsoid(9,
						"EverestIndia1956", 6377301.243, 300.8017), Ellipsoid(10,
						"EverestMalaysia1969", 6377295.664, 300.8017),//Dana has no datum that uses this ellipsoid!
				Ellipsoid(11, "EverestMalay_Sing", 6377304.063, 300.8017), Ellipsoid(12,
						"EverestPakistan", 6377309.613, 300.8017), Ellipsoid(13,
						"Fischer1960Modified", 6378155, 298.3), Ellipsoid(14, "Helmert1906",
						6378200, 298.3), Ellipsoid(15, "Hough1960", 6378270, 297), Ellipsoid(16,
						"Indonesian1974", 6378160, 298.247), Ellipsoid(17, "International1924",
						6378388, 297), Ellipsoid(18, "Krassovsky1940", 6378245, 298.3), Ellipsoid(
						19, "GRS80", 6378137, 298.257222101), Ellipsoid(20, "SouthAmerican1969",
						6378160, 298.25), Ellipsoid(21, "WGS72", 6378135, 298.26), Ellipsoid(22,
						"WGS84", 6378137, 298.257223563) };

#define	eClarke1866	5		//names for ellipsoidId's
#define	eGRS80		19
#define	eWGS72		21
#define	eWGS84		22

class Datum
{
public:
	Datum(int id, const std::string& name, int eid, double dx, double dy, double dz)
	{
		Name = name;
		eId = eid;
		dX = dx;
		dY = dy;
		dZ = dz;
	}
	std::string Name;
	int eId;
	double dX;
	double dY;
	double dZ;
};
static Datum datum[] =
{		//converted from PeterDana website, by Eugene Reimer 2002dec
//	      Id,  Name,			eId,		dX,	dY,	dZ;	//when & where this datum is applicable
		Datum(0, "NAD27_AK", eClarke1866, -5, 135, 172),	//NAD27 for Alaska Excluding Aleutians
		Datum(1, "NAD27_AK_AleutiansE", eClarke1866, -2, 152, 149),	//NAD27 for Aleutians East of 180W
		Datum(2, "NAD27_AK_AleutiansW", eClarke1866, 2, 204, 105),//NAD27 for Aleutians West of 180W
		Datum(3, "NAD27_Bahamas", eClarke1866, -4, 154, 178),//NAD27 for Bahamas Except SanSalvadorIsland
		Datum(4, "NAD27_Bahamas_SanSalv", eClarke1866, 1, 140, 165),//NAD27 for Bahamas SanSalvadorIsland
		Datum(5, "NAD27_AB_BC", eClarke1866, -7, 162, 188),	//NAD27 for Canada Alberta BritishColumbia
		Datum(6, "NAD27_MB_ON", eClarke1866, -9, 157, 184),	//NAD27 for Canada Manitoba Ontario
		Datum(7, "NAD27_NB_NL_NS_QC", eClarke1866, -22, 160, 190),//NAD27 for Canada NewBrunswick Newfoundland NovaScotia Quebec
		Datum(8, "NAD27_NT_SK", eClarke1866, 4, 159, 188),//NAD27 for Canada NorthwestTerritories Saskatchewan
		Datum(9, "NAD27_YT", eClarke1866, -7, 139, 181),	//NAD27 for Canada Yukon
		Datum(10, "NAD27_CanalZone", eClarke1866, 0, 125, 201),	//NAD27 for CanalZone (ER: is that Panama??)
		Datum(11, "NAD27_Cuba", eClarke1866, -9, 152, 178),	//NAD27 for Cuba
		Datum(12, "NAD27_Greenland", eClarke1866, 11, 114, 195),//NAD27 for Greenland (HayesPeninsula)
		Datum(13, "NAD27_Carribean", eClarke1866, -3, 142, 183),//NAD27 for Antigua Barbados Barbuda Caicos Cuba DominicanRep GrandCayman Jamaica Turks
		Datum(14, "NAD27_CtrlAmerica", eClarke1866, 0, 125, 194),//NAD27 for Belize CostaRica ElSalvador Guatemala Honduras Nicaragua
		Datum(15, "NAD27_Canada", eClarke1866, -10, 158, 187),	//NAD27 for Canada
		Datum(16, "NAD27_ConUS", eClarke1866, -8, 160, 176),	//NAD27 for CONUS
		Datum(17, "NAD27_ConUS_East", eClarke1866, -9, 161, 179),//NAD27 for CONUS East of Mississippi Including Louisiana Missouri Minnesota
		Datum(18, "NAD27_ConUS_West", eClarke1866, -8, 159, 175),//NAD27 for CONUS West of Mississippi Excluding Louisiana Missouri Minnesota
		Datum(19, "NAD27_Mexico", eClarke1866, -12, 130, 190),	//NAD27 for Mexico
		Datum(20, "NAD83_AK", eGRS80, 0, 0, 0),	//NAD83 for Alaska Excluding Aleutians
		Datum(21, "NAD83_AK_Aleutians", eGRS80, -2, 0, 4),	//NAD83 for Aleutians
		Datum(22, "NAD83_Canada", eGRS80, 0, 0, 0),	//NAD83 for Canada
		Datum(23, "NAD83_ConUS", eGRS80, 0, 0, 0),	//NAD83 for CONUS
		Datum(24, "NAD83_Hawaii", eGRS80, 1, 1, -1),	//NAD83 for Hawaii
		Datum(25, "NAD83_Mexico_CtrlAmerica", eGRS80, 0, 0, 0),	//NAD83 for Mexico CentralAmerica
		Datum(26, "WGS72", eWGS72, 0, 0, 0),	//WGS72 for world
		Datum(27, "WGS84", eWGS84, 0, 0, 0)	//WGS84 for world
		};
#define	dNAD27_MB_ON	6		//names for datumId's
#define	dNAD27_Canada	15
#define	dNAD83_Canada	22
#define	dNAD83_ConUS	23
#define	dWGS84		27

void
DatumConvert(int dIn, double LatIn, double LongIn, double HtIn, int dTo, double& LatTo,
		double& LongTo, double& HtTo)
{
	// converts LatLongHt in datum dIn, to LatLongHt in datum dTo;  2002dec: by Eugene Reimer, from PeterDana equations.
	// Lat and Long params are in degrees;  North latitudes and East longitudes are positive;  Height is in meters;
	// ==This approach to Datum-conversion is a waste of time;  to get acceptable accuracy a large table is needed -- see NADCON, NTv2...
	double a, ee, N, X, Y, Z, EE, p, b, t;

	//--transform to XYZ, using the "In" ellipsoid
	//LongIn += 180;
	a = ellip[datum[dIn].eId].EquatorialRadius;
	ee = ellip[datum[dIn].eId].eccSquared;
	N = a / sqrt(1 - ee * sin(LatIn * deg2rad) * sin(LatIn * deg2rad));
	X = (N + HtIn) * cos(LatIn * deg2rad) * cos(LongIn * deg2rad);
	Y = (N + HtIn) * cos(LatIn * deg2rad) * sin(LongIn * deg2rad);
	Z = (N * (1 - ee) + HtIn) * sin(LatIn * deg2rad);

	//--apply delta-terms dX dY dZ
	//cout<<"	X:" <<X <<" Y:" <<Y <<" Z:" <<Z;		//==DEBUG
	X += datum[dIn].dX - datum[dTo].dX;
	Y += datum[dIn].dY - datum[dTo].dY;
	Z += datum[dIn].dZ - datum[dTo].dZ;
	//cout<<"	X:" <<X <<" Y:" <<Y <<" Z:" <<Z;		//==DEBUG

	//--transform back to LatLongHeight, using the "To" ellipsoid
	a = ellip[datum[dTo].eId].EquatorialRadius;
	ee = ellip[datum[dTo].eId].eccSquared;
	EE = ee / (1 - ee);
	p = sqrt(X * X + Y * Y);
	b = a * sqrt(1 - ee);
	t = atan(Z * a / (p * b));
	LatTo = atan((Z + EE * b * sin(t) * sin(t) * sin(t)) / (p - ee * a * cos(t) * cos(t) * cos(t)));
	LongTo = atan(Y / X);
	HtTo = p / cos(LatTo) - a / sqrt(1 - ee * sin(LatTo) * sin(LatTo));
	LatTo *= rad2deg;
	LongTo *= rad2deg;
	LongTo -= 180;
}

void
LLtoUTM(int eId, double Lat, double Long, double& Northing, double& Easting, int& Zone)
{
	// converts LatLong to UTM coords;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.
	// Lat and Long are in degrees;  North latitudes and East Longitudes are positive.
	double a = ellip[eId].EquatorialRadius;
	double ee = ellip[eId].eccSquared;
	Long -= int((Long + 180) / 360) * 360;			//ensure longitude within -180.00..179.9
	double N, T, C, A, M;
	double LatRad = Lat * deg2rad;
	double LongRad = Long * deg2rad;

	Zone = int((Long + 186) / 6);
	if (Lat >= 56.0 && Lat < 64.0 && Long >= 3.0 && Long < 12.0)
		Zone = 32;
	if (Lat >= 72.0 && Lat < 84.0)
	{			//Special zones for Svalbard
		if (Long >= 0.0 && Long < 9.0)
			Zone = 31;
		else if (Long >= 9.0 && Long < 21.0)
			Zone = 33;
		else if (Long >= 21.0 && Long < 33.0)
			Zone = 35;
		else if (Long >= 33.0 && Long < 42.0)
			Zone = 37;
	}
	double LongOrigin = Zone * 6 - 183;			//origin in middle of zone
	double LongOriginRad = LongOrigin * deg2rad;

	double EE = ee / (1 - ee);

	N = a / sqrt(1 - ee * sin(LatRad) * sin(LatRad));
	T = tan(LatRad) * tan(LatRad);
	C = EE * cos(LatRad) * cos(LatRad);
	A = cos(LatRad) * (LongRad - LongOriginRad);

	M = a
			* ((1 - ee / 4 - 3 * ee * ee / 64 - 5 * ee * ee * ee / 256) * LatRad
					- (3 * ee / 8 + 3 * ee * ee / 32 + 45 * ee * ee * ee / 1024) * sin(2 * LatRad)
					+ (15 * ee * ee / 256 + 45 * ee * ee * ee / 1024) * sin(4 * LatRad)
					- (35 * ee * ee * ee / 3072) * sin(6 * LatRad));

	Easting = k0 * N
			* (A + (1 - T + C) * A * A * A / 6
					+ (5 - 18 * T + T * T + 72 * C - 58 * EE) * A * A * A * A * A / 120) + 500000.0;

	Northing = k0
			* (M
					+ N * tan(LatRad)
							* (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
									+ (61 - 58 * T + T * T + 600 * C - 330 * EE) * A * A * A * A * A
											* A / 720));
}

void
UTMtoLL(int eId, double Northing, double Easting, int Zone, double& Lat, double& Long)
{
	// converts UTM coords to LatLong;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.
	// Lat and Long are in degrees;  North latitudes and East Longitudes are positive.
	double a = ellip[eId].EquatorialRadius;
	double ee = ellip[eId].eccSquared;
	double EE = ee / (1 - ee);
	double e1 = (1 - sqrt(1 - ee)) / (1 + sqrt(1 - ee));
	double N1, T1, C1, R1, D, M, mu, phi1Rad;
	double x = Easting - 500000.0;			//remove 500,000 meter offset for longitude
	double y = Northing;
	double LongOrigin = Zone * 6 - 183;			//origin in middle of zone

	M = y / k0;
	mu = M / (a * (1 - ee / 4 - 3 * ee * ee / 64 - 5 * ee * ee * ee / 256));

	phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
			+ (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
			+ (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
	N1 = a / sqrt(1 - ee * sin(phi1Rad) * sin(phi1Rad));
	T1 = tan(phi1Rad) * tan(phi1Rad);
	C1 = EE * cos(phi1Rad) * cos(phi1Rad);
	R1 = a * (1 - ee) / pow(1 - ee * sin(phi1Rad) * sin(phi1Rad), 1.5);
	D = x / (N1 * k0);

	Lat = phi1Rad
			- (N1 * tan(phi1Rad) / R1)
					* (D * D / 2
							- (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * EE) * D * D * D * D / 24
							+ (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * EE - 3 * C1 * C1) * D
									* D * D * D * D * D / 720);
	Lat *= rad2deg;
	Long =
			(D - (1 + 2 * T1 + C1) * D * D * D / 6
					+ (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * EE + 24 * T1 * T1) * D * D * D * D
							* D / 120) / cos(phi1Rad);
	Long = LongOrigin + Long * rad2deg;
}

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UTMTOLATLONG_H_ */
