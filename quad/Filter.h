
#ifndef Filter_h
#define Filter_h


#define NZEROS 5
#define NPOLES 5
#define GAIN   2.291257749e+03



class Filter
{

  public:

    Filter();
    float update(float input);

    
  private:

    float inv_gain;
    float xv[NZEROS+1], yv[NPOLES+1];
    

};

#endif
