// Update occupancy with new scan data (source)

#include "occ_update.hh"

// Update occupancy hash map with data per pose
void vox_update(std::unordered_map<ind, free_unk_data> & opp,
                std::unordered_map<ind, occ_data> & occ,
                std::unordered_map<ind, free_unk_data> & freev,
                std::unordered_map<ind, free_unk_data> & unk,
                int pose_ind,
                int max_depth,
                float max_thresh
                )
{

  std::unordered_map <ind, free_unk_data> ::iterator it;
  for(it = opp.begin(); it != opp.end(); ++it)
  {
    ind cpt = {it->first.x, it->first.y, it->first.z};

    ++occ[cpt].hits;
    occ[cpt].probability = (float)occ[cpt].hits / (float)(pose_ind+1);
  }

  // Reset temporary occupancy map
  opp.clear();

  float mean_probability = prob_update(occ
                                       );

  // Retrieve bounding box
  corners bounds(mean_probability);
  corners prior_bounds(mean_probability);
  prior_bounds.min_x = 0;
  prior_bounds.max_x = 0;
  prior_bounds.min_y = 0;
  prior_bounds.max_y = 0;
  prior_bounds.min_z = 0;
  prior_bounds.max_z = 0;

  for(int i = 0; i < max_depth; ++i)
  {
    float thresh = max_thresh - (float)(2*i)

    bounds.get_corners(occ,
                       thresh
                       );

    bounds.even_out();

    // parse bbx for unk with coarsest bbx
    parse_bbx(occ,
              freev,
              unk,
              bounds,
              prior_bounds,
              (max_depth-i)
              );

    prior_bounds = bounds;

  }

  unk.clear();
  for(int x_i = bounds.min_x; x_i <= bounds.max_x; ++x_i)
  {
    for(int y_i = bounds.min_y; y_i <= bounds.max_y; ++y_i)
    {
      for(int z_i = bounds.min_z; z_i <= bounds.max_z; ++z_i)
      {
        if(freev.count( {x_i, y_i, z_i} ) == 0)
        {
          if(occ.count( {x_i, y_i, z_i} ) == 0)
          {
            // Store unknown voxel indecies
            ++unk[ {x_i, y_i, z_i} ].hits;
          }
        }
      }
    }
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

void parse_bbx(std::unordered_map<ind, occ_data> & occ,
               std::unordered_map<ind, free_unk_data> & freev,
               std::unordered_map<ind, free_unk_data> & unk,
               corners &bounds,
               corners &prior_bounds,
               int depthl
               )
{
  int x_i = bounds.min_x;
  while(x_i <= bounds.max_x)
  {
    if((x_i > prior_bounds.min_x) && (x_i < prior_bounds.max_x)) { x_i = prior_bounds.max_x; }
    int y_i = bounds.min_y;
    while(y_i <= bounds.max_y)
    {
      if((y_i > prior_bounds.min_y) && (y_i < prior_bounds.max_y)) { y_i = prior_bounds.max_y; }
      int z_i = bounds.min_z;
      while(z_i <= bounds.max_z)
      {
        if((z_i > prior_bounds.min_z) && (z_i < prior_bounds.max_z)) { z_i = prior_bounds.max_z; }

        // ADJUST EXTENT OF VOXELS

        ++z_i;
      }
      ++y_i
    }
    ++z_i;
  }
}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Update occupancy masking with probabilistic calculation
float prob_update(std::unordered_map<ind, occ_data> & occ
                  )
{

  float mean_probability = 0.0f;
  std::unordered_map <ind, occ_data> ::iterator it;

  for(it = occ.begin(); it != occ.end(); ++it)
  {
    mean_probability = mean_probability + it->second.probability;
  }

  mean_probability = mean_probability / occ.size();

  // Cull occupied space according to occupancy probability
  std::unordered_map <ind, occ_data> ::iterator it_cull;
  float threshold = 1.5f;

  for(it_cull = occ.begin(); it_cull != occ.end(); ++it_cull)
  {
    ind cpt = {it_cull->first.x, it_cull->first.y, it_cull->first.z};

    if((occ[cpt].probability > (threshold * mean_probability)))
    {
      occ[cpt].mask = true;
    }
    else if((occ[cpt].probability < (threshold * mean_probability)))
    {
      occ[cpt].mask = false;
    }
  }

  return mean_probability;

}
