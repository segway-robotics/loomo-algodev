#ifndef TF2_VECTOR3_H
#define TF2_VECTOR3_H


#include "Scalar.h"
#include "../impl/min_max.h"

namespace ninebot_tf
{

#define Vector3Data Vector3DoubleData
#define Vector3DataName "Vector3DoubleData"




/**@brief ninebot_tf::Vector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when ninebot_tf::Vector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized TF2SIMD version that keeps the data in registers
 */
ATTRIBUTE_ALIGNED16(class) Vector3
{
public:

#if defined (__SPU__) && defined (__CELLOS_LV2__)
		tf2Scalar	m_floats[4];
public:
	inline const vec_float4&	get128() const
	{
		return *((const vec_float4*)&m_floats[0]);
	}
public:
#else //__CELLOS_LV2__ __SPU__
#ifdef TF2_USE_SSE // _WIN32
	union {
		__m128 mVec128;
		tf2Scalar	m_floats[4];
	};
	inline	__m128	get128() const
	{
		return mVec128;
	}
	inline	void	set128(__m128 v128)
	{
		mVec128 = v128;
	}
#else
	tf2Scalar	m_floats[4];
#endif
#endif //__CELLOS_LV2__ __SPU__

	public:

  /**@brief No initialization constructor */
	inline Vector3() {}

 
	
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
	inline Vector3(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z)
	{
		m_floats[0] = x;
		m_floats[1] = y;
		m_floats[2] = z;
		m_floats[3] = tf2Scalar(0.);
	}

/**@brief Add a vector to this one 
 * @param The vector to add to this one */
	inline Vector3& operator+=(const Vector3& v)
	{

		m_floats[0] += v.m_floats[0]; m_floats[1] += v.m_floats[1];m_floats[2] += v.m_floats[2];
		return *this;
	}


  /**@brief Sutf2ract a vector from this one
   * @param The vector to sutf2ract */
	inline Vector3& operator-=(const Vector3& v) 
	{
		m_floats[0] -= v.m_floats[0]; m_floats[1] -= v.m_floats[1];m_floats[2] -= v.m_floats[2];
		return *this;
	}
  /**@brief Scale the vector
   * @param s Scale factor */
	inline Vector3& operator*=(const tf2Scalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s;m_floats[2] *= s;
		return *this;
	}

  /**@brief Inversely scale the vector 
   * @param s Scale factor to divide by */
	inline Vector3& operator/=(const tf2Scalar& s) 
	{
		tf2FullAssert(s != tf2Scalar(0.0));
		return *this *= tf2Scalar(1.0) / s;
	}

  /**@brief Return the dot product
   * @param v The other vector in the dot product */
	inline tf2Scalar dot(const Vector3& v) const
	{
		return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] +m_floats[2] * v.m_floats[2];
	}

  /**@brief Return the length of the vector squared */
	inline tf2Scalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the vector */
	inline tf2Scalar length() const
	{
		return tf2Sqrt(length2());
	}

  /**@brief Return the distance squared between the ends of this and another vector
   * This is symantically treating the vector like a point */
	inline tf2Scalar distance2(const Vector3& v) const;

  /**@brief Return the distance between the ends of this and another vector
   * This is symantically treating the vector like a point */
	inline tf2Scalar distance(const Vector3& v) const;

  /**@brief Normalize this vector 
   * x^2 + y^2 + z^2 = 1 */
	inline Vector3& normalize() 
	{
		return *this /= length();
	}

  /**@brief Return a normalized version of this vector */
	inline Vector3 normalized() const;

  /**@brief Rotate this vector 
   * @param wAxis The axis to rotate about 
   * @param angle The angle to rotate by */
	inline Vector3 rotate( const Vector3& wAxis, const tf2Scalar angle ) const;

  /**@brief Return the angle between this and another vector
   * @param v The other vector */
	inline tf2Scalar angle(const Vector3& v) const 
	{
		tf2Scalar s = tf2Sqrt(length2() * v.length2());
		tf2FullAssert(s != tf2Scalar(0.0));
		return tf2Acos(dot(v) / s);
	}
  /**@brief Return a vector will the absolute values of each element */
	inline Vector3 absolute() const 
	{
		return Vector3(
			tf2Fabs(m_floats[0]), 
			tf2Fabs(m_floats[1]), 
			tf2Fabs(m_floats[2]));
	}
  /**@brief Return the cross product between this and another vector 
   * @param v The other vector */
	inline Vector3 cross(const Vector3& v) const
	{
		return Vector3(
			m_floats[1] * v.m_floats[2] -m_floats[2] * v.m_floats[1],
			m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
			m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]);
	}

	inline tf2Scalar triple(const Vector3& v1, const Vector3& v2) const
	{
		return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) + 
			m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) + 
			m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
	}

  /**@brief Return the axis with the smallest value 
   * Note return values are 0,1,2 for x, y, or z */
	inline int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2);
	}

  /**@brief Return the axis with the largest value 
   * Note return values are 0,1,2 for x, y, or z */
	inline int maxAxis() const 
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
	}

	inline int furthestAxis() const
	{
		return absolute().minAxis();
	}

	inline int closestAxis() const 
	{
		return absolute().maxAxis();
	}

	inline void setInterpolate3(const Vector3& v0, const Vector3& v1, tf2Scalar rt)
	{
		tf2Scalar s = tf2Scalar(1.0) - rt;
		m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
		m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
		m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
	}

	inline Vector3 lerp(const Vector3& v, const tf2Scalar& t) const 
	{
		return Vector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
			m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
			m_floats[2] + (v.m_floats[2] -m_floats[2]) * t);
	}

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
	inline Vector3& operator*=(const Vector3& v)
	{
		m_floats[0] *= v.m_floats[0]; m_floats[1] *= v.m_floats[1];m_floats[2] *= v.m_floats[2];
		return *this;
	}

	 /**@brief Return the x value */
		inline const tf2Scalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
		inline const tf2Scalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
		inline const tf2Scalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
		inline void	setX(tf2Scalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
		inline void	setY(tf2Scalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
		inline void	setZ(tf2Scalar z) {m_floats[2] = z;};
  /**@brief Set the w value */
		inline void	setW(tf2Scalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
		inline const tf2Scalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
		inline const tf2Scalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
		inline const tf2Scalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
		inline const tf2Scalar& w() const { return m_floats[3]; }

	//inline tf2Scalar&       operator[](int i)       { return (&m_floats[0])[i];	}      
	//inline const tf2Scalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator tf2Scalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	inline	operator       tf2Scalar *()       { return &m_floats[0]; }
	inline	operator const tf2Scalar *() const { return &m_floats[0]; }

	inline	bool	operator==(const Vector3& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	inline	bool	operator!=(const Vector3& other) const
	{
		return !(*this == other);
	}

	 /**@brief Set each element to the max of the current values and the values of another Vector3
   * @param other The other Vector3 to compare with 
   */
		inline void	setMax(const Vector3& other)
		{
			tf2SetMax(m_floats[0], other.m_floats[0]);
			tf2SetMax(m_floats[1], other.m_floats[1]);
			tf2SetMax(m_floats[2], other.m_floats[2]);
			tf2SetMax(m_floats[3], other.w());
		}
  /**@brief Set each element to the min of the current values and the values of another Vector3
   * @param other The other Vector3 to compare with 
   */
		inline void	setMin(const Vector3& other)
		{
			tf2SetMin(m_floats[0], other.m_floats[0]);
			tf2SetMin(m_floats[1], other.m_floats[1]);
			tf2SetMin(m_floats[2], other.m_floats[2]);
			tf2SetMin(m_floats[3], other.w());
		}

		inline void 	setValue(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = tf2Scalar(0.);
		}

		void	getSkewSymmetricMatrix(Vector3* v0,Vector3* v1,Vector3* v2) const
		{
			v0->setValue(0.		,-z()		,y());
			v1->setValue(z()	,0.			,-x());
			v2->setValue(-y()	,x()	,0.);
		}

		void	setZero()
		{
			setValue(tf2Scalar(0.),tf2Scalar(0.),tf2Scalar(0.));
		}

		inline bool isZero() const 
		{
			return m_floats[0] == tf2Scalar(0) && m_floats[1] == tf2Scalar(0) && m_floats[2] == tf2Scalar(0);
		}

		inline bool fuzzyZero() const 
		{
			return length2() < TF2SIMD_EPSILON;
		}

		inline	void	serialize(struct	Vector3Data& dataOut) const;

		inline	void	deSerialize(const struct	Vector3Data& dataIn);

		inline	void	serializeFloat(struct	Vector3FloatData& dataOut) const;

		inline	void	deSerializeFloat(const struct	Vector3FloatData& dataIn);

		inline	void	serializeDouble(struct	Vector3DoubleData& dataOut) const;

		inline	void	deSerializeDouble(const struct	Vector3DoubleData& dataIn);

};

/**@brief Return the sum of two vectors (Point symantics)*/
inline Vector3 
operator+(const Vector3& v1, const Vector3& v2) 
{
	return Vector3(v1.m_floats[0] + v2.m_floats[0], v1.m_floats[1] + v2.m_floats[1], v1.m_floats[2] + v2.m_floats[2]);
}

/**@brief Return the elementwise product of two vectors */
inline Vector3 
operator*(const Vector3& v1, const Vector3& v2) 
{
	return Vector3(v1.m_floats[0] * v2.m_floats[0], v1.m_floats[1] * v2.m_floats[1], v1.m_floats[2] * v2.m_floats[2]);
}

/**@brief Return the difference between two vectors */
inline Vector3 
operator-(const Vector3& v1, const Vector3& v2)
{
	return Vector3(v1.m_floats[0] - v2.m_floats[0], v1.m_floats[1] - v2.m_floats[1], v1.m_floats[2] - v2.m_floats[2]);
}
/**@brief Return the negative of the vector */
inline Vector3 
operator-(const Vector3& v)
{
	return Vector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
}

/**@brief Return the vector scaled by s */
inline Vector3 
operator*(const Vector3& v, const tf2Scalar& s)
{
	return Vector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
}

/**@brief Return the vector scaled by s */
inline Vector3 
operator*(const tf2Scalar& s, const Vector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
inline Vector3
operator/(const Vector3& v, const tf2Scalar& s)
{
	tf2FullAssert(s != tf2Scalar(0.0));
	return v * (tf2Scalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
inline Vector3
operator/(const Vector3& v1, const Vector3& v2)
{
	return Vector3(v1.m_floats[0] / v2.m_floats[0],v1.m_floats[1] / v2.m_floats[1],v1.m_floats[2] / v2.m_floats[2]);
}

/**@brief Return the dot product between two vectors */
inline tf2Scalar 
tf2Dot(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
inline tf2Scalar
tf2Distance2(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
inline tf2Scalar
tf2Distance(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
inline tf2Scalar
tf2Angle(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
inline Vector3 
tf2Cross(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.cross(v2); 
}

inline tf2Scalar
tf2Triple(const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
	return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector 
 * @param v2 The other vector 
 * @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
inline Vector3 
lerp(const Vector3& v1, const Vector3& v2, const tf2Scalar& t)
{
	return v1.lerp(v2, t);
}



inline tf2Scalar Vector3::distance2(const Vector3& v) const
{
	return (v - *this).length2();
}

inline tf2Scalar Vector3::distance(const Vector3& v) const
{
	return (v - *this).length();
}

inline Vector3 Vector3::normalized() const
{
	return *this / length();
} 

inline Vector3 Vector3::rotate( const Vector3& wAxis, const tf2Scalar angle ) const
{
	// wAxis must be a unit lenght vector

	Vector3 o = wAxis * wAxis.dot( *this );
	Vector3 x = *this - o;
	Vector3 y;

	y = wAxis.cross( *this );

	return ( o + x * tf2Cos( angle ) + y * tf2Sin( angle ) );
}

class tf2Vector4 : public Vector3
{
public:

	inline tf2Vector4() {}


	inline tf2Vector4(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,const tf2Scalar& w) 
		: Vector3(x,y,z)
	{
		m_floats[3] = w;
	}


	inline tf2Vector4 absolute4() const 
	{
		return tf2Vector4(
			tf2Fabs(m_floats[0]), 
			tf2Fabs(m_floats[1]), 
			tf2Fabs(m_floats[2]),
			tf2Fabs(m_floats[3]));
	}



	tf2Scalar	getW() const { return m_floats[3];}


		inline int maxAxis4() const
	{
		int maxIndex = -1;
		tf2Scalar maxVal = tf2Scalar(-TF2_LARGE_FLOAT);
		if (m_floats[0] > maxVal)
		{
			maxIndex = 0;
			maxVal = m_floats[0];
		}
		if (m_floats[1] > maxVal)
		{
			maxIndex = 1;
			maxVal = m_floats[1];
		}
		if (m_floats[2] > maxVal)
		{
			maxIndex = 2;
			maxVal =m_floats[2];
		}
		if (m_floats[3] > maxVal)
		{
			maxIndex = 3;
		}
		
		
		

		return maxIndex;

	}


	inline int minAxis4() const
	{
		int minIndex = -1;
		tf2Scalar minVal = tf2Scalar(TF2_LARGE_FLOAT);
		if (m_floats[0] < minVal)
		{
			minIndex = 0;
			minVal = m_floats[0];
		}
		if (m_floats[1] < minVal)
		{
			minIndex = 1;
			minVal = m_floats[1];
		}
		if (m_floats[2] < minVal)
		{
			minIndex = 2;
			minVal =m_floats[2];
		}
		if (m_floats[3] < minVal)
		{
			minIndex = 3;
		}
		
		return minIndex;

	}


	inline int closestAxis4() const 
	{
		return absolute4().maxAxis4();
	}

	
 

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		

/*		void getValue(tf2Scalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] =m_floats[2];
		}
*/
/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
		inline void	setValue(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,const tf2Scalar& w)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3]=w;
		}


};


///tf2SwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
inline void	tf2SwapScalarEndian(const tf2Scalar& sourceVal, tf2Scalar& destVal)
{
	unsigned char* dest = (unsigned char*) &destVal;
	unsigned char* src  = (unsigned char*) &sourceVal;
	dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
}
///tf2SwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
inline void	tf2SwapVector3Endian(const Vector3& sourceVec, Vector3& destVec)
{
	for (int i=0;i<4;i++)
	{
		tf2SwapScalarEndian(sourceVec[i],destVec[i]);
	}

}

///tf2UnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
inline void	tf2UnSwapVector3Endian(Vector3& vector)
{

	Vector3	swappedVec;
	for (int i=0;i<4;i++)
	{
		tf2SwapScalarEndian(vector[i],swappedVec[i]);
	}
	vector = swappedVec;
}

inline void tf2PlaneSpace1 (const Vector3& n, Vector3& p, Vector3& q)
{
  if (tf2Fabs(n.z()) > TF2SIMDSQRT12) {
    // choose p in y-z plane
    tf2Scalar a = n[1]*n[1] + n[2]*n[2];
    tf2Scalar k = tf2RecipSqrt (a);
    p.setValue(0,-n[2]*k,n[1]*k);
    // set q = n x p
    q.setValue(a*k,-n[0]*p[2],n[0]*p[1]);
  }
  else {
    // choose p in x-y plane
    tf2Scalar a = n.x()*n.x() + n.y()*n.y();
    tf2Scalar k = tf2RecipSqrt (a);
    p.setValue(-n.y()*k,n.x()*k,0);
    // set q = n x p
    q.setValue(-n.z()*p.y(),n.z()*p.x(),a*k);
  }
}


struct	Vector3FloatData
{
	float	m_floats[4];
};

struct	Vector3DoubleData
{
	double	m_floats[4];

};

inline	void	Vector3::serializeFloat(struct	Vector3FloatData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

inline void	Vector3::deSerializeFloat(const struct	Vector3FloatData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = tf2Scalar(dataIn.m_floats[i]);
}


inline	void	Vector3::serializeDouble(struct	Vector3DoubleData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

inline void	Vector3::deSerializeDouble(const struct	Vector3DoubleData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = tf2Scalar(dataIn.m_floats[i]);
}


inline	void	Vector3::serialize(struct	Vector3Data& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = m_floats[i];
}

inline void	Vector3::deSerialize(const struct	Vector3Data& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = dataIn.m_floats[i];
}

}

#endif //TF2_VECTOR3_H
