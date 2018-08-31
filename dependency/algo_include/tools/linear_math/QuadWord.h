#ifndef TF2SIMD_QUADWORD_H
#define TF2SIMD_QUADWORD_H

#include "Scalar.h"
#include "../impl/min_max.h"


#if defined (__CELLOS_LV2) && defined (__SPU__)
#include <altivec.h>
#endif

namespace ninebot_tf
{
/**@brief The QuadWord class is base class for Vector3 and Quaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
#ifndef USE_LIBSPE2
ATTRIBUTE_ALIGNED16(class) QuadWord
#else
class QuadWord
#endif
{
protected:

#if defined (__SPU__) && defined (__CELLOS_LV2__)
	union {
		vec_float4 mVec128;
		tf2Scalar	m_floats[4];
	};
public:
	vec_float4	get128() const
	{
		return mVec128;
	}
protected:
#else //__CELLOS_LV2__ __SPU__
	tf2Scalar	m_floats[4];
#endif //__CELLOS_LV2__ __SPU__

	public:
  

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
		inline void	setZ(tf2Scalar z) { m_floats[2] = z;};
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

	inline	bool	operator==(const QuadWord& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	inline	bool	operator!=(const QuadWord& other) const
	{
		return !(*this == other);
	}

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		inline void 	setValue(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = 0.f;
		}
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
  /**@brief No initialization constructor */
		inline QuadWord()
		//	:m_floats[0](tf2Scalar(0.)),m_floats[1](tf2Scalar(0.)),m_floats[2](tf2Scalar(0.)),m_floats[3](tf2Scalar(0.))
		{
		}
 
  /**@brief Three argument constructor (zeros w)
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		inline QuadWord(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z)		
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = 0.0f;
		}

/**@brief Initializing constructor
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
		inline QuadWord(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,const tf2Scalar& w) 
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
		}

  /**@brief Set each element to the max of the current values and the values of another QuadWord
   * @param other The other QuadWord to compare with 
   */
		inline void	setMax(const QuadWord& other)
		{
			tf2SetMax(m_floats[0], other.m_floats[0]);
			tf2SetMax(m_floats[1], other.m_floats[1]);
			tf2SetMax(m_floats[2], other.m_floats[2]);
			tf2SetMax(m_floats[3], other.m_floats[3]);
		}
  /**@brief Set each element to the min of the current values and the values of another QuadWord
   * @param other The other QuadWord to compare with 
   */
		inline void	setMin(const QuadWord& other)
		{
			tf2SetMin(m_floats[0], other.m_floats[0]);
			tf2SetMin(m_floats[1], other.m_floats[1]);
			tf2SetMin(m_floats[2], other.m_floats[2]);
			tf2SetMin(m_floats[3], other.m_floats[3]);
		}



};

}
#endif //TF2SIMD_QUADWORD_H
