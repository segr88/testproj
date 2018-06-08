#ifndef SLIDEAVG_H
#define SLIDEAVG_H



template <typename T>
class Slideavg {
	public:
		Slideavg(int _maxn);
		~Slideavg(); 
		void add(T v);
		int setnumof(int _n);
		T getAVG();
	private:
		int n;
		int maxn;
		int ifront;
		int iback;
	  int size;
		T sum;
		T *q;
		void push(T v);
		void pop();
	  T front();
};


#endif
