#ifndef VECTOR_H_
#define	VECTOR_H_


/**
 * @brief 3d vector
 * 
 */
class Vector
{
public:
	Vector() { x = y = z = 0.; }
	Vector(double v1, double v2, double v3) {
		x = v1; y = v2; z = v3;
	}
	Vector(const Vector& v) {
		x = v.x;  y = v.y; z = v.z;
	}

	void setX(double x) { this->x = x; }
	void setY(double y) { this->y = y; }
	void setZ(double z) { this->z = z; }
	
	
	/**
	 * @brief get the magnitude
	 * 
	 * @return double 
	 */
	double magnitude() const;
	
	/**
	 * @brief vector normalization
	 * 
	 */
	void normalize();

	/**
	 * @brief assignment
	 * 
	 * @param v 
	 * @return const Vector& 
	 */
	const Vector& operator = (const Vector& v);
	const Vector& operator = (double v);

    const Vector operator + (const Vector& v) const;
    const Vector operator - (const Vector& v) const;
    const Vector operator * (double s) const;  
    const Vector operator / (double s) const;
    
    const Vector& operator += (const Vector& v);
    const Vector& operator -= (const Vector& v);
    const Vector& operator += (double delta);
    const Vector& operator -= (double delta);

    /**
     * @brief dot product
     * 
     * @param v 
     * @return double 
     */
	double operator * (const Vector& v) const;

	/**
	 * @brief cross product
	 * 
	 * @param V 
	 * @return const Vector 
	 */
	const Vector operator ^ (const Vector& V) const;
	const Vector operator - ();

	friend const Vector operator * (double scale, const Vector& v);

public:
	double x, y, z;
};

#endif
