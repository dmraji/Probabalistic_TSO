#ifndef ray_caster_hh
#define ray_caster_hh

using namespace std;

class ray_caster
{

  public:
    ray_caster(float**,
               int,
               float**,
               int,
               float**
               );

    ~ray_caster();

  public:
    void int_kill(int,
                  int**
                  );

    std::vector<float> linspace(float,
                                float,
                                int
                                );

  private:
    float diff_x, diff_y, diff_z;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    int x_pts, y_pts, z_pts, tot_pts;
    int box_ind;
    std::vector<float> spaced_x, spaced_y, spaced_z;


};

#endif
