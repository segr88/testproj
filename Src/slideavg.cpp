
#include "slideavg.h"


template <typename T>
Slideavg<T>::Slideavg(int _maxn) 
{
	maxn = _maxn;
	if (maxn<1) maxn=1;
	n = maxn;
	q = new T[maxn+1]; 
	ifront = 0;
	iback = 1;
	size = 0;
	sum = 0;
}

template <typename T>
Slideavg<T>::~Slideavg() 
{
	delete q;
}

template <typename T>
void Slideavg<T>::add(T v)
{
	push(v);
	sum += v;
	while ((size>n)&&(size>0))
	{
		sum -= front();
		pop();
	}
}

template <typename T>
int Slideavg<T>::setnumof(int _n)
{
	if (_n<=maxn) n=_n;
	return n;
}

template <typename T>
T Slideavg<T>::getAVG()
{
	if (size>0){
		return sum/(T)size;
	} 
	else 
		return 0;
}

//my queue

template <typename T>
void Slideavg<T>::push(T v)
{
	if(size<maxn)
	{
		q[iback++]=v;
		if(iback>maxn) iback=0;
		size++;
	}
}

template <typename T>
void Slideavg<T>::pop()
{
	if(size>0)
	{
		T temp = q[ifront++];
		if(ifront>maxn) ifront=0;
		size--;
	}
}

template <typename T>
T Slideavg<T>::front()
{
	return q[ifront];
}