#ifndef ray_caster_hh
#define ray_caster_hh

using namespace std;

class ray_caster
{

  public:
    ray_caster(vector< vector<float> > &,
               int,
               vector< vector<float> > &,
               int,
               float**
               );

    ~ray_caster();

  public:
    int init_zero(int,
                  int**
                  );

    void int_kill(int,
                  int**
                  );

    std::vector<float> linspace(float,
                                float,
                                int
                                );

    std::vector<float> g_range(float,
                               float,
                               float
                               );

  private:
    float resolution;
    float diff_x, diff_y, diff_z;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    float mag, increment;
    float pt_x, pt_y, pt_z;
    float prox_2;
    int x_pts, y_pts, z_pts, tot_pts;
    int box_ind;
    int cloud_cut, cloud_chunk_len;
    int max_ax;
    int increment_index;
    std::vector<float> spaced_x, spaced_y, spaced_z;
    // std::vector<float> x_neg, x_pos, y_neg, y_pos, z_neg, z_pos;



};

#endif
