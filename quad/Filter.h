/*
  Filter.h - Low pass filter
  Created by Romain Goussault <romain.goussault@gmail.com>
  Based on http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  afloat with this program. If not, see <http://www.gnu.org/licenses/>. 
*/


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
