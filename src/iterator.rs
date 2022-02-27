use delaunator::{Point, Triangulation, next_halfedge};
use crate::{Voronoi, utils::dist2};

use super::{EMPTY};

/// Iterator that walks through all the edges connected to a provided starting point.
/// The iteration happens in a clock-wise manner.
///
/// Note: this really only returns edges for triangles around the site. On the convex hull, the rightmost edge will not be returned because there is no incoming rightmost edge.
#[derive(Clone)]
pub struct EdgesAroundSiteIterator<'t> {
    triangulation: &'t Triangulation,
    start: usize,
    next: usize
}

impl<'t> EdgesAroundSiteIterator<'t> {
    /// Creates iterator based on a incoming edge to a site.
    /// This must be the left-most incoming edge to the site to avoid early iteration stop around the convex hull.
    pub fn new(triangulation: &'t Triangulation, incoming_edge: usize) -> Self {
        Self {
            triangulation,
            start: incoming_edge,
            next: incoming_edge
        }
    }
}

impl<'t> Iterator for EdgesAroundSiteIterator<'t> {
    type Item = usize;
    /// Walks all half-edges around the starting point and returning the associated surrounding incoming edges
    fn next(&mut self) -> Option<Self::Item> {
        let incoming = self.next;

        if incoming != EMPTY {
            let outgoing = next_halfedge(incoming);

            // then take the oposite half-edge, it will be the incoming edge for the opposite triangle
            self.next = self.triangulation.halfedges[outgoing];

            // if we are back to the begining, there is nothing else to do
            if self.next == self.start {
                self.next = EMPTY
            }

            Some(incoming)
        } else {
            None
        }
    }
}

/// Iterates over sites that neighbor another site.
#[derive(Clone)]
pub struct NeighborSiteIterator<'t> {
    iter: EdgesAroundSiteIterator<'t>,
    //triangulation: &'t Triangulation,
    last: usize,
    //source_site: usize,
}

impl<'t> NeighborSiteIterator<'t> {
    /// Creates iterator based on the site.
    pub fn new(voronoi: &'t Voronoi, site: usize) -> Self {
        Self::with_triangulation(voronoi.triangulation(), &voronoi.site_to_incoming_leftmost_halfedge, site)
    }

    /// Creates iterator based on the site.
    pub fn with_triangulation(triangulation: &'t Triangulation, site_to_incoming_leftmost_halfedge: &'t Vec<usize>, site: usize) -> Self {
        let &incoming_leftmost_edge = site_to_incoming_leftmost_halfedge.get(site).expect("Site does not exist");
        Self {
            iter: EdgesAroundSiteIterator::new(&triangulation, incoming_leftmost_edge),
            //triangulation,
            //source_site: site,
            last: EMPTY,
        }
    }


    // fn has_common_edge(&self, neighbor_site: usize) -> bool {
    //     // FIXME: this is probably wrong - 2 common vertices are needed for a common edge, but when hull cells are closed
    //     // same vertice positions are duplicated with new indexes, so simple index comparison here does not work
    //     // this depends on the fact that at least one of the vertices in common is a original circumcenter
    //     self.voronoi.cells[self.source_site].iter().any(|t| self.voronoi.cells[neighbor_site].contains(t))
    // }
}

impl<'t> Iterator for NeighborSiteIterator<'t> {
    type Item = usize;
    /// Walks all half-edges around the starting point and returning the associated surrounding sites
    fn next(&mut self) -> Option<Self::Item> {
        let mut site = None;
        while let Some(incoming) = self.iter.next() {
            self.last = incoming;

            // get site from where the incoming edge came from
            let neighbor_site = self.iter.triangulation.triangles[incoming];
            site = Some(neighbor_site);
            break;

            // voronoi sites are topologically connected to other sites based if there is a delaunay edge between then
            // however clipping may remove that edge and the associated cells in the voronoi diagram may not share a common edge
            // this may happen if current and neighbor cells are in the hull
            // TODO checking if cells are on the hull may cost more than just always checking for common edge
            // FIXME: add a way to distinguish topological neighbors from visual (clipped) neighbors
            // let neighbor_cell = self.voronoi.cell(neighbor_site);
            // if neighbor_cell.is_on_hull() && current_cell.is_on_hull() {
            //     if self.has_common_edge(neighbor_site) {
            //         // site and neig// site and neighbor is on hull and they are connected because they share a non-clipped edge
            //         site = Some(neighbor_site);
            //         break;
            //     } else {
            //         // neighbors on hull do not share an edge (clipped)
            //         continue;
            //     }
            // } else {
            //     site = Some(neighbor_site);
            //     break;
            // }
        }

        if site.is_some() {
            site
        } else if self.last != EMPTY {
            // check if there is a next site on the hull
            let outgoing = next_halfedge(self.last);
            if self.iter.triangulation.halfedges[outgoing] == EMPTY {
                // this means we are on the hull and reached the rightmost outgoing edge
                self.last = EMPTY;

                // FIXME this logic is confusing - ideally this would be merged with the loop above
                // let neighbor_site = self.iter.triangulation.triangles[next_halfedge(outgoing)];
                // if self.has_common_edge(neighbor_site) {
                //     Some(neighbor_site)
                // } else {
                //     None
                // }
                let neighbor_site = self.iter.triangulation.triangles[next_halfedge(outgoing)];
                Some(neighbor_site)
            } else {
                // this means site is not on hull, and we have already iterated over all neighbors
                None
            }
        } else {
            None
        }
    }
}

/// Iterator that produces a path between two points in the Voronoi diagram that uses a greed approach to minimizes a cost function.
///
/// A cost function is provided that calculates the cost of an edge; edges for all neighbors are evaluated and the least costly is taken.
/// The process is evaluated for the next cell in the path until no edge can be taken that costs less than f64::MAX.
/// If the destionation point is not contained in the Voronoi diagram, the final cell in the path will be the
/// closest to the destination point.
#[derive(Clone)]
pub struct CellPathIterator<'t, F> {
    site: usize,
    cost_fn: F,
    triangulation: &'t Triangulation,
    site_to_incoming_leftmost_halfedge: &'t Vec<usize>
}

impl<'t, F> CellPathIterator<'t, F> {
    /// Creates iterator based on the starting site and cost function.
    pub fn new(voronoi: &'t Voronoi, site: usize, cost_fn: F) -> Self {
        assert!(site < voronoi.sites.len(), "site {} does not exist", site);

        Self::with_triangulation(voronoi.triangulation(), &voronoi.site_to_incoming_leftmost_halfedge, site, cost_fn)
    }

    pub fn with_triangulation(triangulation: &'t Triangulation, site_to_incoming_leftmost_halfedge: &'t Vec<usize>, site: usize, cost_fn: F) -> Self {
        Self {
            site,
            cost_fn,
            triangulation,
            site_to_incoming_leftmost_halfedge
        }
    }
}

impl<'t, F> Iterator for CellPathIterator<'t, F>
    where F : Fn(usize, usize) -> f64 {
    type Item = usize;

    /// Walks current site neighbor and find the next site in the path
    fn next(&mut self) -> Option<Self::Item> {
        let current_site = self.site;

        if current_site != EMPTY {
            // take the neighbor with least cost
            let next = NeighborSiteIterator::with_triangulation(self.triangulation, self.site_to_incoming_leftmost_halfedge, current_site)
                .map(|n| (n, (self.cost_fn)(current_site, n)))
                .min_by(|(_, cost0), (_, cost1)| cost0.partial_cmp(cost1).unwrap());

            // if next neighbor cost is less than f64::MAX, then we can move to it - it is next in the path
            if let Some((n, cost)) = next {
                if cost < f64::MAX {
                    self.site = n;
                } else {
                    // reached end
                    self.site = EMPTY;
                }
            } else {
                // reached end
                self.site = EMPTY;
            }

            Some(current_site)
        } else {
            None
        }
    }
}

/// Produces an iterator that calculates the shortest path from ```start_site``` to a ```dest``` point.
///
/// If destination point is outside voronoi diagram, then the closest point to destination in the voronoi diagram will be returned.
pub fn shortest_path_iter<'v>(voronoi: &'v Voronoi, start_site: usize, dest: Point) -> impl Iterator<Item = usize> + 'v {
    shortest_path_iter_from_triangulation(voronoi.triangulation(), &voronoi.sites(), &voronoi.site_to_incoming_leftmost_halfedge, start_site, dest)
}

pub (crate) fn shortest_path_iter_from_triangulation<'t>(triangulation: &'t Triangulation, sites: &'t Vec<Point>, site_to_incoming_leftmost_halfedge: &'t Vec<usize>, start_site: usize, dest: Point) -> impl Iterator<Item = usize> + 't {
    CellPathIterator::with_triangulation(triangulation, site_to_incoming_leftmost_halfedge, start_site, move |curr, next| {
        // calculate distance
        let dist_to_dest = dist2(&sites[curr], &dest);
        let dist_from_next = dist2(&sites[next], &dest);

        if dist_to_dest <= dist_from_next {
            // if current is closer to dest than next is, make cost to travel to next impossibly high
            f64::MAX
        } else {
            // cost is distance
            dist_from_next
        }
    })
}

#[cfg(test)]
mod test {
    use delaunator::Point;
    use crate::VoronoiBuilder;
    use super::*;

    #[test]
    fn iter_neighbors_hull_test() {
        let sites = vec![Point { x: -0.5, y: 0.0 }, Point { x: 0.5, y: 0.0 }, Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 }];
        let v = VoronoiBuilder::default()
            .set_sites(sites)
            .build()
            .unwrap();
        let neighbors: Vec<usize> = NeighborSiteIterator::new(&v, 0).collect();
        assert_eq!(neighbors.len(), 3, "There are 3 neighboring sites");
        assert_eq!(neighbors[0], 4);
        assert_eq!(neighbors[1], 2);
        assert_eq!(neighbors[2], 3);
    }

    #[test]
    fn iter_neighbors_inner_test() {
        let sites = vec![Point { x: -0.5, y: 0.0 }, Point { x: 0.5, y: 0.0 }, Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 }];
        let v = VoronoiBuilder::default()
            .set_sites(sites)
            .build()
            .unwrap();
        let neighbors: Vec<usize> = NeighborSiteIterator::new(&v, 2).collect();
        assert_eq!(neighbors.len(), 4, "There are 4 neighboring sites");
        assert_eq!(neighbors[0], 3);
        assert_eq!(neighbors[1], 0);
        assert_eq!(neighbors[2], 4);
        assert_eq!(neighbors[3], 1);
    }

    // FIXME https://github.com/andreesteve/voronoice/issues/9
    // #[test]
    // fn iter_neighbors_edge_clipped_by_box_test() {
    //     // points 0 and 1 are neighbors if the bounding box is a square of side 7
    //     // when bounding box is a square of side 2, the edge between 0 and 1 is removed and 2 becomes a cell in between 0 and 1

    //     // another problematic set is:
    //     /*
    //         [-0.5, -0.8],
    //         [0, -0.5],
    //         [0.2, -0.5],
    //         [0.3, -0.5],
    //     */

    //     // need to find a way to remove delauney neighbors whose voronoi edges were clipped out
    //     // comparing edge is one way, comparing circumcenters is another https://github.com/d3/d3-delaunay/pull/98/files
    //     let sites = vec![Point { x: -1.0, y: -1.0 }, Point { x: 0.0, y: -1.0 }, Point { x: -0.45, y: -0.95 }];
    //     let v = VoronoiBuilder::default()
    //         .set_sites(sites.clone())
    //         .build()
    //         .unwrap();
    //     let mut neighbors = NeighborSiteIterator::new(&v, 0);
    //     assert_eq!(Some(2), neighbors.next());
    //     assert_eq!(None, neighbors.next());
    // }

    #[test]
    fn iter_cell_path_test() {
        let sites = vec![
            Point { x: -0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 },
            Point { x: 0.2, y: 0.0 }, Point { x: 0.2, y: 0.5 }, Point { x: 0.2, y: -0.5 },
            Point { x: 0.5, y: 0.0 },
        ];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let mut path = shortest_path_iter(&v, 0, sites.last().unwrap().clone());
        assert_eq!(Some(0), path.next());
        assert_eq!(Some(1), path.next());
        assert_eq!(Some(4), path.next());
        assert_eq!(Some(7), path.next());
        assert_eq!(None, path.next());
    }

    // FIXME: https://github.com/andreesteve/voronoice/issues/9
    // #[test]
    // fn iter_cell_path_test_2() {
    //     let sites = vec![
    //         Point { x: -0.9, y: -0.9 },
    //         Point { x: -0.5, y: -0.8 }, Point { x: -0.8, y: -0.6 },
    //         Point { x: -0.5, y: -0.5 }, Point { x: -0.5, y: 0.0 },
    //         Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 },
    //         Point { x: 0.2, y: 0.0 }, Point { x: 0.2, y: 0.5 }, Point { x: 0.2, y: -0.5 },
    //         Point { x: 0.3, y: 0.0 }, Point { x: 0.3, y: 0.5 }, Point { x: 0.3, y: -0.5 },
    //         Point { x: 0.5, y: 0.0 },
    //         Point { x: 0.5, y: 0.5 },
    //     ];
    //     let v = VoronoiBuilder::default()
    //         .set_sites(sites.clone())
    //         .build()
    //         .unwrap();
    //         let mut path = shortest_path_iter(&v, 0, sites.last().unwrap().clone());
    //     assert_eq!(Some(0), path.next());
    //     assert_eq!(Some(1), path.next());
    //     // this fails because the point 13 is a neighbor of 1; this is technically true if we expand the bounding box to a large value
    //     // 13 and 1 share a voronoi edge, but that edge is clipped by the bounding box
    //     assert_eq!(Some(3), path.next());
    //     assert_eq!(Some(5), path.next());
    //     assert_eq!(Some(8), path.next());
    //     assert_eq!(None, path.next());
    // }
}