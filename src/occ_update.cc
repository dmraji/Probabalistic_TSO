// Update occupancy with new scan data (source)

#include "occ_update.hh"

#include "il_math.hh"

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
    float thresh;
    if(i != (max_depth-1))
    {
      thresh = max_thresh - (float)(2*i);
    }
    else
    {
      // Ensure all voxels are caught in the final depth depth level
      thresh = 0.0f;
    }

    bounds.get_corners(occ,
                       thresh
                       );

    bounds.even_out();


    parse_bbx(occ,
              freev,
              unk,
              bounds,
              prior_bounds,
              max_depth-i,
              max_depth
              );

    prior_bounds = bounds;

  }

  // unk.clear();
  // for(int x_i = bounds.min_x; x_i <= bounds.max_x; ++x_i)
  // {
  //   for(int y_i = bounds.min_y; y_i <= bounds.max_y; ++y_i)
  //   {
  //     for(int z_i = bounds.min_z; z_i <= bounds.max_z; ++z_i)
  //     {
  //       if(freev.count( {x_i, y_i, z_i} ) == 0)
  //       {
  //         if(occ.count( {x_i, y_i, z_i} ) == 0)
  //         {
  //           // Store unknown voxel indecies
  //           ++unk[ {x_i, y_i, z_i} ].hits;
  //         }
  //       }
  //     }
  //   }
  // }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Scan bounding box in accordance with probabilistically-determined depth level
void parse_bbx(std::unordered_map<ind, occ_data> & occ,
               std::unordered_map<ind, free_unk_data> & freev,
               std::unordered_map<ind, free_unk_data> & unk,
               corners &bounds,
               corners &prior_bounds,
               int depthl,
               int max_depth
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

        ind cind = {x_i, y_i, z_i};

        // Store unknown voxel indecies at maximum depth
        if(depthl == max_depth)
        {
          unk.clear();

          if(freev.count(cind) == 0)
          {
            if(occ.count(cind) == 0)
            {

              ++unk[cind].hits;
            }
          }
        }
        adjust_report rep = adj_extent(freev,
                                       cind,
                                       depthl
                                       );
        if(!rep.exist)
        {
          adjust_report rep = adj_extent(occ,
                                         cind,
                                         depthl
                                         );
        }
        if(!rep.exist)
        {
          adjust_report rep = adj_extent(unk,
                                         cind,
                                         depthl
                                         );

          if(rep.exist)
          {
            switch(rep.inc)
            {
              case 1:
                ind ind_par = cind.get_parent_ind(unk[cind].sr_extent
                                                  );

                std::vector<ind> children;
                ind_par.get_child_inds(children,
                                       unk[cind].sr_extent
                                       );

                unk[ind_par].sr_extent = unk[cind].sr_extent;
                // Compressing children into parent
                for(int child_i = 0; child_i < 8; ++child_i)
                {
                  if(unk.count(children[child_i]) != 0)
                  {
                    unk[ind_par].hits += unk[children[child_i]].hits;
                    unk.erase(children[child_i]);
                  }
                }

                break;
              case -1:
                std::vector<ind> children;
                cind.get_child_inds(children,
                                    unk[cind].sr_extent
                                    );

                // Subdivide parent, split assets between children;
                for(int child_i = 0; child_i < 8; ++child_i)
                {
                  unk[children[child_i]].hits = unk[cind].hits / 8;
                  unk[children[child_i]].sr_extent = unk[cind].sr_extent;
                }

                break;
              case 0:
                break;
            }
          }
        }

        ++z_i;
      }
      ++y_i;
    }
    ++x_i;
  }


}

// Update voxel size and record changes in report data structure
adjust_report adj_extent(std::unordered_map<ind, free_unk_data> & vox,
                         ind cind,
                         int depthl
                         )
{
  if(vox.count(cind) != 0)
  {
    if(vox[cind].sr_extent < depthl)
    {
      vox[cind].sr_extent *= 2;
      return {true, 1};
    }
    else if(vox[cind].sr_extent > depthl)
    {
      vox[cind].sr_extent /= 2;
      return {true, -1};
    }
    else { return {true, 0}; }
  }
  else { return {false, 0}; }
}

adjust_report adj_extent(std::unordered_map<ind, occ_data> & vox,
                         ind cind,
                         int depthl
                         )
{
  if(vox.count(cind) != 0)
  {
    if(vox[cind].sr_extent < depthl)
    {
      vox[cind].sr_extent *= 2;
      return {true, 1};
    }
    else if(vox[cind].sr_extent > depthl)
    {
      vox[cind].sr_extent /= 2;
      return {true, -1};
    }
    else { return {true, 0}; }
  }
  else { return {false, 0}; }
}



void prune(std::unordered_map<ind, occ_data> & occ,
           std::unordered_map<ind, free_unk_data> & freev,
           std::unordered_map<ind, free_unk_data> & unk
           )
{
  for(auto it = occ.cbegin(); it != occ.cend();)
  {
    // if(it->second.prior_extent < it->second.sr_extent) { occ.erase(it++); }
    // else { ++it; }
    continue;
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

    if((occ[cpt].probability * (1/cub(occ[cpt].sr_extent))) > (threshold * mean_probability))
    {
      occ[cpt].mask = true;
    }
    else if((occ[cpt].probability * (1/cub(occ[cpt].sr_extent))) < (threshold * mean_probability))
    {
      occ[cpt].mask = false;
    }
  }

  return mean_probability;

}
