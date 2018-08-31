#ifndef IMPL_MINMAX_H
#define IMPL_MINMAX_H

template <class T>
inline const T& tf2Min(const T& a, const T& b)
{
  return a < b ? a : b ;
}

template <class T>
inline const T& tf2Max(const T& a, const T& b)
{
  return  a > b ? a : b;
}

template <class T>
inline const T& GEN_clamped(const T& a, const T& lb, const T& ub)
{
	return a < lb ? lb : (ub < a ? ub : a);
}

template <class T>
inline void tf2SetMin(T& a, const T& b)
{
    if (b < a)
	{
		a = b;
	}
}

template <class T>
inline void tf2SetMax(T& a, const T& b)
{
    if (a < b)
	{
		a = b;
	}
}

template <class T>
inline void GEN_clamp(T& a, const T& lb, const T& ub)
{
	if (a < lb)
	{
		a = lb;
	}
	else if (ub < a)
	{
		a = ub;
	}
}

#endif
