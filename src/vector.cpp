#include "vector.h"
#include <math.h>

double Vector::magnitude() const
{
	return sqrt(x * x + y * y + z * z);
}

void Vector::normalize()
{
	double m = magnitude();
	x = x / m;
	y = y / m;
	z = z / m;
}

const Vector& Vector::operator = (const Vector& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

const Vector& Vector::operator = (double v)
{
	x = y = z = v;
	return *this;
}

const Vector Vector::operator + (const Vector& v) const
{
	Vector vr;
	vr.x = x + v.x;
	vr.y = y + v.y;
	vr.z = z + v.z;
	return vr;
}

const Vector Vector::operator - (const Vector& v) const
{
	Vector vr;
	vr.x = x - v.x;
	vr.y = y - v.y;
	vr.z = z - v.z;
	return vr;
}

const Vector Vector::operator * (double s) const
{
	Vector vr;
	vr.x = x * s;
	vr.y = y * s;
	vr.z = z * s;
	return vr;
}
 
const Vector Vector::operator / (double s) const
{
	Vector vr;
	vr.x = x / s;
	vr.y = y / s;
	vr.z = z / s;
	return vr;
}


const Vector& Vector::operator += (const Vector& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

const Vector& Vector::operator -= (const Vector& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

const Vector& Vector::operator += (double delta)
{
	x += delta;
	y += delta;
	z += delta;
	return *this;
}

const Vector& Vector::operator -= (double delta)
{
	x -= delta;
	y -= delta;
	z -= delta;
	return *this;
}

double Vector::operator * (const Vector& v) const
{
	return x * v.x + y * v.y + z * v.z;
}

const Vector Vector::operator ^ (const Vector& v) const
{
	Vector vr;
    vr.x = y * v.z - v.y * z;
	vr.y = v.x * z - x * v.z;
	vr.z = x * v.y - y * v.x;
	return vr;
}


const Vector Vector::operator - ()
{
	Vector vr;
	vr.x = -x;
	vr.y = -y;
	vr.z = -z;
	return vr;
}

const Vector operator * (double scale, const Vector& v)
{
	 return v * scale;
}

