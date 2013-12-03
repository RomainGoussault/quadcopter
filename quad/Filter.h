
#ifndef Filter_h
#define Filter_h


#define NZEROS 8
#define NPOLES 8
#define GAIN   3.901575440e+06



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
