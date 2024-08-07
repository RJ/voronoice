use delaunator::{next_halfedge, Point, Triangulation};

use super::EMPTY;
use crate::{
    utils::{self, dist2, site_of_incoming},
    ConvexBoundary, Voronoi,
};

/// Iterator that walks through all the edges connected to a provided starting point.
/// The iteration happens in a clock-wise manner.
///
/// Note: this really only returns edges for triangles around the site. On the convex hull, the rightmost edge will not be returned because there is no incoming rightmost edge.
#[derive(Clone, Debug)]
pub struct EdgesAroundSiteIterator<'t> {
    triangulation: &'t Triangulation,
    start: usize,
    next: usize,
}

impl<'t> EdgesAroundSiteIterator<'t> {
    /// Creates iterator based on a incoming edge to a site.
    /// This must be the left-most incoming edge to the site to avoid early iteration stop around the convex hull.
    pub fn new(triangulation: &'t Triangulation, incoming_edge: usize) -> Self {
        Self {
            triangulation,
            start: incoming_edge,
            next: incoming_edge,
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

/// Iterates over sites that are topologically adjacent.
///
/// Topological neighbors are sites that share a delaunay edge between them.
/// To take into account the effect of voronoi edge clipping use [NeighborSiteIterator]
/// Sites are returned clockwise.
#[derive(Clone, Debug)]
pub struct TopologicalNeighborSiteIterator<'t> {
    iter: EdgesAroundSiteIterator<'t>,
    last_incoming: usize,
}

impl<'t> TopologicalNeighborSiteIterator<'t> {
    /// Creates iterator based on the site.
    pub fn new<T: ConvexBoundary>(voronoi: &'t Voronoi<T>, site: usize) -> Self {
        Self::with_triangulation(
            voronoi.triangulation(),
            &voronoi.site_to_incoming_leftmost_halfedge,
            site,
        )
    }

    /// Creates iterator based on the site.
    pub fn with_triangulation(
        triangulation: &'t Triangulation,
        site_to_incoming_leftmost_halfedge: &'t Vec<usize>,
        site: usize,
    ) -> Self {
        let &incoming_leftmost_edge = site_to_incoming_leftmost_halfedge
            .get(site)
            .expect("Site does not exist");
        Self {
            iter: EdgesAroundSiteIterator::new(&triangulation, incoming_leftmost_edge),
            last_incoming: EMPTY,
        }
    }
}

impl<'t> Iterator for TopologicalNeighborSiteIterator<'t> {
    type Item = usize;
    /// Get the next neighboring site.
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(incoming_edge) = self.iter.next() {
            self.last_incoming = incoming_edge;
            Some(self.iter.triangulation.triangles[incoming_edge])
        } else if self.last_incoming != EMPTY {
            // if we are on hull, the last neighbor is not returned by EdgesAroundSiteIterator because there is no halfedge
            let outgoing = next_halfedge(self.last_incoming);
            self.last_incoming = EMPTY;
            if self.iter.triangulation.halfedges[outgoing] == EMPTY {
                // on hull edge
                Some(site_of_incoming(self.iter.triangulation, outgoing))
            } else {
                // not on hull
                None
            }
        } else {
            None
        }
    }
}

/// Iterates over sites that are adjacent in the voronoi diagram.
///
/// This iterator expands on [TopologicalNeighborSiteIterator] by taking into account the clipping effect of voronoi edges to decide whether two sites are neighbors.
/// In this iterator, two sites are considered neighbors if their voronoi cells share a common edge in the voronoi graph.
/// Sites on the hull may get disconnected by this classification if their common voronoi edge gets clipped away because it lies beyond the bounding geometry.
#[derive(Clone, Debug)]
pub struct NeighborSiteIterator<'t, T: ConvexBoundary> {
    voronoi: &'t Voronoi<T>,
    topo_neighbor_iter: TopologicalNeighborSiteIterator<'t>,
    site: usize,
}

impl<'t, T: ConvexBoundary> NeighborSiteIterator<'t, T> {
    pub fn new(voronoi: &'t Voronoi<T>, site: usize) -> Self {
        Self {
            voronoi,
            topo_neighbor_iter: TopologicalNeighborSiteIterator::new(voronoi, site),
            site,
        }
    }
}

impl<'t, T: ConvexBoundary> Iterator for NeighborSiteIterator<'t, T> {
    type Item = usize;

    /// Get the next neighboring site.
    fn next(&mut self) -> Option<Self::Item> {
        let prev_last_incoming = self.topo_neighbor_iter.last_incoming;

        if let Some(neighbor) = self.topo_neighbor_iter.next() {
            // if first neighbor and on hull, need special check for clipping
            if prev_last_incoming == EMPTY
                && self.voronoi.triangulation.halfedges[self.topo_neighbor_iter.last_incoming]
                    == EMPTY
            {
                if utils::has_common_voronoi_edge(self.voronoi, self.site, neighbor) {
                    Some(neighbor)
                } else {
                    // not connected on the voronoi diagram
                    // move to next
                    self.next()
                }
            } else if self.topo_neighbor_iter.last_incoming == EMPTY {
                // last neighbor, need speciail check for clipping
                if utils::has_common_voronoi_edge(self.voronoi, self.site, neighbor) {
                    Some(neighbor)
                } else {
                    None
                }
            } else {
                Some(neighbor)
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
#[derive(Clone, Debug)]
pub struct CellPathIterator<'t, F> {
    site: usize,
    cost_fn: F,
    triangulation: &'t Triangulation,
    site_to_incoming_leftmost_halfedge: &'t Vec<usize>,
}

impl<'t, F> CellPathIterator<'t, F> {
    /// Creates iterator based on the starting site and cost function.
    pub fn new<T: ConvexBoundary>(voronoi: &'t Voronoi<T>, site: usize, cost_fn: F) -> Self {
        assert!(site < voronoi.sites.len(), "site {} does not exist", site);

        Self::with_triangulation(
            voronoi.triangulation(),
            &voronoi.site_to_incoming_leftmost_halfedge,
            site,
            cost_fn,
        )
    }

    pub fn with_triangulation(
        triangulation: &'t Triangulation,
        site_to_incoming_leftmost_halfedge: &'t Vec<usize>,
        site: usize,
        cost_fn: F,
    ) -> Self {
        Self {
            site,
            cost_fn,
            triangulation,
            site_to_incoming_leftmost_halfedge,
        }
    }
}

impl<'t, F> Iterator for CellPathIterator<'t, F>
where
    F: Fn(usize, usize) -> f64,
{
    type Item = usize;

    /// Walks current site neighbor and find the next site in the path
    fn next(&mut self) -> Option<Self::Item> {
        let current_site = self.site;

        if current_site != EMPTY {
            // take the neighbor with least cost
            let next = TopologicalNeighborSiteIterator::with_triangulation(
                self.triangulation,
                self.site_to_incoming_leftmost_halfedge,
                current_site,
            )
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
pub fn shortest_path_iter<'v, T: ConvexBoundary>(
    voronoi: &'v Voronoi<T>,
    start_site: usize,
    dest: Point,
) -> impl Iterator<Item = usize> + 'v {
    shortest_path_iter_from_triangulation(
        voronoi.triangulation(),
        &voronoi.sites(),
        &voronoi.site_to_incoming_leftmost_halfedge,
        start_site,
        dest,
    )
}

pub(crate) fn shortest_path_iter_from_triangulation<'t>(
    triangulation: &'t Triangulation,
    sites: &'t Vec<Point>,
    site_to_incoming_leftmost_halfedge: &'t Vec<usize>,
    start_site: usize,
    dest: Point,
) -> impl Iterator<Item = usize> + 't {
    CellPathIterator::with_triangulation(
        triangulation,
        site_to_incoming_leftmost_halfedge,
        start_site,
        move |curr, next| {
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
        },
    )
}

#[cfg(test)]
mod test {
    use delaunator::Point;

    use super::*;
    use crate::{utils::test::assert_list_eq, BoundingBox, VoronoiBuilder};

    #[test]
    fn iter_neighbors_hull_test() {
        let sites = vec![
            Point { x: -0.5, y: 0.0 },
            Point { x: 0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: 0.5 },
            Point { x: 0.0, y: -0.5 },
        ];
        let v = VoronoiBuilder::<BoundingBox>::default()
            .set_sites(sites)
            .build()
            .unwrap();

        let neighbors: Vec<usize> = TopologicalNeighborSiteIterator::new(&v, 0).collect();
        assert_eq!(neighbors.len(), 3, "There are 3 neighboring sites");
        assert_eq!(neighbors[0], 4);
        assert_eq!(neighbors[1], 2);
        assert_eq!(neighbors[2], 3);
    }

    #[test]
    fn iter_neighbors_inner_test() {
        let sites = vec![
            Point { x: -0.5, y: 0.0 },
            Point { x: 0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: 0.5 },
            Point { x: 0.0, y: -0.5 },
        ];
        let v = VoronoiBuilder::<BoundingBox>::default()
            .set_sites(sites)
            .build()
            .unwrap();
        let neighbors: Vec<usize> = TopologicalNeighborSiteIterator::new(&v, 2).collect();
        assert_eq!(neighbors.len(), 4, "There are 4 neighboring sites");
        assert_eq!(neighbors[0], 3);
        assert_eq!(neighbors[1], 0);
        assert_eq!(neighbors[2], 4);
        assert_eq!(neighbors[3], 1);
    }

    #[test]
    fn iter_neighbors_edge_clipped_by_box_test() -> std::io::Result<()> {
        let voronoi = utils::test::new_voronoi_builder_from_asset("degenerated8.json")?
            .build()
            .expect("Some voronoi expected");

        let neighbors = NeighborSiteIterator::new(&voronoi, 0).collect::<Vec<_>>();
        assert_list_eq(&[3, 2], &neighbors, "Visible neighbors of 0");

        let neighbors = NeighborSiteIterator::new(&voronoi, 1).collect::<Vec<_>>();
        assert_list_eq(&[2], &neighbors, "Visible neighbors of 1");

        Ok(())
    }

    #[test]
    fn iter_cell_path_test() {
        let sites = vec![
            Point { x: -0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: 0.5 },
            Point { x: 0.0, y: -0.5 },
            Point { x: 0.2, y: 0.0 },
            Point { x: 0.2, y: 0.5 },
            Point { x: 0.2, y: -0.5 },
            Point { x: 0.5, y: 0.0 },
        ];
        let v = VoronoiBuilder::<BoundingBox>::default()
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
}
