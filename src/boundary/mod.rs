mod bounding_box;
mod convex_polygon;

use std::{fmt::Display, str::FromStr};

use super::Point;

pub use bounding_box::BoundingBox;
pub use convex_polygon::ConvexPolygon;

/// Defines how Voronoi generation will handle clipping of Voronoi cell edges within the convex boundary.
///
/// Clipping is necessary to guarantee that all Voronoi vertices are within the convex boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClipBehavior {
    /// No clipping will be performed. Any sites outside the convex boundary will still be used for diagram generation.
    None,

    /// Removes any sites outside the convex boundary, but does not perform any further clipping of Voronoi cells that may end up outside of the convex boundary.
    RemoveSitesOutsideBoundaryOnly,

    /// Removes sites outside convex boundary and clips any Voronoi cell edges that fall outside of the convex boundary.
    Clip,
}

impl Default for ClipBehavior {
    fn default() -> Self {
        ClipBehavior::Clip
    }
}

impl Display for ClipBehavior {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl FromStr for ClipBehavior {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "None" => Ok(Self::None),
            "RemoveSitesOutsideBoundaryOnly" => Ok(Self::RemoveSitesOutsideBoundaryOnly),
            "Clip" => Ok(Self::Clip),
            _ => Err("Invalid option".to_string())
        }
    }
}

pub trait ConvexBoundary: std::fmt::Debug + Default + Clone {
    /// Gets a slice of vertices oriented counter-clockwise.
    fn vertices(&self) -> &[Point];

    /// Returns whether a given point is inside (or on the edges) of the boundary.
    fn is_inside(&self, point: &Point) -> bool;

    /// Same as [Self::is_inside()], but returns false if point is on the box edge.
    #[inline]
    fn is_exclusively_inside(&self, point: &Point) -> bool {
        self.is_inside(point) && self.which_edge(point).is_none()
    }

    /// Returns the index of the vertex representing the edge ```point``` is on.
    ///
    /// [Self::vertices()] can be called to get the list of corners.
    ///
    /// # Example
    /// In a [BoundingBox], if `point` is on middle of the top edge, the top left corner will be returned.
    fn which_edge(&self, point: &Point) -> Option<usize>;

    fn next_edge(&self, edge: usize) -> usize {
        (edge + 1) % self.vertices().len()
    }

    /// Intersects a line represented by points 'a' and 'b' with the boundary and returns the two intersecting points, or None
    fn intersect_line(&self, a: &Point, b: &Point) -> (Option<Point>, Option<Point>);

    /// Intersects a ray with the bounding box. The first intersection is returned first.
    fn project_ray(&self, point: &Point, direction: &Point) -> (Option<Point>, Option<Point>) {
        let b = Point { x: point.x + direction.x, y: point.y + direction.y };
        let (a, b) = self.intersect_line(point, &b);
        order_points_on_ray(point, direction, a, b)
    }
}

/// Given a ray defined by `point` and `direction`, and two points `a` and `b` on such ray, returns a tuple (w, z) where point <= w <= z.
/// If either `a` or `b` are smaller than `point`, None is returned.
pub (crate) fn order_points_on_ray(point: &Point, direction: &Point, a: Option<Point>, b: Option<Point>) -> (Option<Point>, Option<Point>) {
    match (a,b) {
        (None, None) => { // no points, no intersection
            (None, None)
        }
        (Some(va), Some(vb)) => { // both a and b are reachable
            // point a and b are just a scalar times direction, so we can compare any non-zero
            // direction component, use largest
            let (d, da, db) = if direction.x.abs() > direction.y.abs() {
                // use x for comparison
                (direction.x, va.x - point.x, vb.x - point.x)
            } else {
                (direction.y, va.y - point.y, vb.y - point.y)
            };

            match (d.signum() == da.signum(), d.signum() == db.signum()) {
                (true, true) => {
                    if da.abs() > db.abs() {
                        // b is closer
                        (Some(vb), Some(va))
                    } else {
                        // a is closer
                        (Some(va), Some(vb))
                    }
                },
                (true, false) => { // only a reachable
                    (Some(va), None)
                },
                (false, true) => { // only b reachably
                    (Some(vb), None)
                },
                (false, false) => { // neither a nor b is reachable, no intersection
                    (None, None)
                }
            }
        },
        (Some(va), None) => {
            if direction.x.signum() == va.x.signum() && direction.y.signum() == va.y.signum() {
                // a is in the right direction
                (Some(va), None)
            } else {
                // a can't be reached
                (None, None)
            }
        },
        (None, Some(vb)) => {
            if direction.x.signum() == vb.x.signum() && direction.y.signum() == vb.y.signum() {
                // b is in the right direction
                (Some(vb), None)
            } else {
                // b can't be reached
                (None, None)
            }
        }
    }
}