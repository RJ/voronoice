use super::{ConvexBoundary, Point};
use crate::{utils::EQ_EPSILON, VoronoiceError};
const ISECT_EPSILON: f64 = 5e-4;

#[derive(Debug, Clone)]
pub struct VoroConvexPolygon {
    vertices: Vec<Point>,
}

impl Default for VoroConvexPolygon {
    fn default() -> Self {
        Self::new(vec![
            Point { x: -1.0, y: -1.0 },
            Point { x: -1.0, y: 1.0 },
            Point { x: 1.0, y: 1.0 },
            Point { x: 1.0, y: -1.0 },
        ])
        .unwrap()
    }
}

impl ConvexBoundary for VoroConvexPolygon {
    fn vertices(&self) -> &[Point] {
        &self.vertices
    }

    fn is_inside(&self, point: &Point) -> bool {
        for (a, b) in self
            .vertices
            .iter()
            .zip(self.vertices.iter().cycle().skip(1))
        {
            if robust::orient2d(a.into(), b.into(), point.into()) > 0.0 {
                return false;
            }
        }

        true
    }

    fn which_edge(&self, point: &Point) -> Result<usize, VoronoiceError> {
        // Iterate over each pair of consecutive vertices (edges) in the polygon
        for (i, (a, b)) in self
            .vertices
            .iter()
            .zip(self.vertices.iter().cycle().skip(1))
            .enumerate()
        {
            // Calculate the bounding box of the current edge
            // add a small slop to the bounding box to account for floating point precision
            // which causes failures when converting from f32 in my game..
            // this is only broadphase bbox check anyway, worst case we do a little extra work..
            let slop = 0.0001;
            let edge_left = f64::min(a.x, b.x) - slop;
            let edge_top = f64::min(a.y, b.y) - slop;
            let edge_right = f64::max(a.x, b.x) + slop;
            let edge_bottom = f64::max(a.y, b.y) + slop;
            #[cfg(feature = "debug_logs")]
            {
                println!("Testing {point:?} against edge: ({i}, ({a:?}, {b:?}))")
            }
            // Check if the point lies within the bounding box of the edge
            if edge_left <= point.x
                && point.x <= edge_right
                && edge_top <= point.y
                && point.y <= edge_bottom
            {
                #[cfg(feature = "debug_logs")]
                {
                    println!("🟩 Inside bounding box for edge {a:?} -> {b:?}");
                }
                // Calculate the orientation of the point with respect to the edge
                let val = robust::orient2d(a.into(), b.into(), point.into());
                // Debug logs to print the orientation value and epsilon comparison
                let is_good = val.abs() <= ISECT_EPSILON;
                #[cfg(feature = "debug_logs")]
                {
                    println!("Point inside edge bounding box, orient2d value: {val}, epsilon: {ISECT_EPSILON} {}",
                                if is_good { "✅" } else { "❌" });
                    println!(
                        "(|val| - epsilon)/epsilon = {}",
                        (val.abs() - ISECT_EPSILON) / ISECT_EPSILON,
                    );
                }
                // If the orientation value is within the epsilon threshold,
                // the point lies on the edge, so return the edge index
                if is_good {
                    return Ok((i + 1) % self.vertices.len());
                }
            } else {
                #[cfg(feature = "debug_logs")]
                {
                    println!("🟥 NOT Inside bounding box for edge {a:?} -> {b:?}");
                    let point_x = point.x;
                    let point_y = point.y;
                    println!(
                        "if edge_left({edge_left}) <= point.x({point_x})
 && point.x({point_x}) <= edge_right({edge_right})
 && edge_top({edge_top}) <= point.y({point_y})
 && point.y({point_y}) <= edge_bottom({edge_bottom})"
                    );
                }
            }
        }

        // If the point doesn't lie on any edge, return an error
        // eprintln!("💀 {self:?}\npoint={point:?}");
        Err(VoronoiceError::Unspecified(
            "Point not on any edge (convex polygon)".to_string(),
        ))
    }

    fn intersect_line(&self, a: &Point, b: &Point) -> (Option<Point>, Option<Point>) {
        let mut found = None;
        // TODO: binary search?
        for (v1, v2) in self
            .vertices
            .iter()
            .zip(self.vertices.iter().cycle().skip(1))
        {
            let o_v1 = robust::orient2d(a.into(), b.into(), v1.into());
            let o_v2 = robust::orient2d(a.into(), b.into(), v2.into());
            // Case: line crosses both vertices
            if o_v1.abs() <= EQ_EPSILON && o_v2.abs() <= EQ_EPSILON {
                return (Some(v1.clone()), Some(v2.clone()));
            // Case: line crosses the first vertex
            } else if o_v1.abs() <= EQ_EPSILON {
                if found.is_none() {
                    found = Some(v1.clone());
                } else {
                    return (found, Some(v1.clone()));
                }
                // Ignore the second vertex as it will be checked on the subsequent edge
            }
            // Case: line crosses the edge somewhere between the vertices
            // Given the line L parametrized as L(t) = a + t(b − a),
            // the intersect occurs at t = n · (v1 − a)/n · (b − a),
            // where n is perpendicular to the edge (v2 - v1).
            if o_v1 * o_v2 < 0.0 {
                // Edge normal
                let n_x = v1.y - v2.y;
                let n_y = v2.x - v1.x;
                let t = (n_x * (v1.x - a.x) + n_y * (v1.y - a.y))
                    / (n_x * (b.x - a.x) + n_y * (b.y - a.y));
                let intersect = Point {
                    x: a.x + t * (b.x - a.x),
                    y: a.y + t * (b.y - a.y),
                };
                if found.is_none() {
                    found = Some(intersect);
                } else {
                    return (found, Some(intersect));
                }
            }
        }
        return (found, None);
    }
}

impl VoroConvexPolygon {
    fn is_convex_ccw(vertices: &[Point]) -> bool {
        for ((a, b), c) in vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .zip(vertices.iter().cycle().skip(2))
        {
            if robust::orient2d(a.into(), b.into(), c.into()) > 0.0 {
                return false;
            }
        }
        true
    }

    pub fn new(vertices: Vec<Point>) -> Result<Self, VoronoiceError> {
        if !Self::is_convex_ccw(&vertices) {
            return Err(VoronoiceError::PolygonNotConvexAndCCW);
        }
        Ok(VoroConvexPolygon { vertices })
    }

    pub fn regular(n: i32, radius: f64) -> Result<Self, VoronoiceError> {
        let phase = std::f64::consts::FRAC_PI_2
            + if n % 2 == 0 {
                std::f64::consts::PI / n as f64
            } else {
                0.0
            };
        let step = 2.0 * std::f64::consts::PI / n as f64;
        Self::new(
            (0..n)
                .into_iter()
                .map(|i| Point {
                    x: f64::cos(-(i as f64) * step - phase) * radius,
                    y: f64::sin(-(i as f64) * step - phase) * radius,
                })
                .collect(),
        )
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn dented_box() -> Vec<Point> {
        vec![
            Point { x: 1.0, y: 0.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: -1.0, y: -1.0 },
            Point { x: -1.0, y: 1.0 },
            Point { x: 0.0, y: 1.0 },
        ]
    }

    #[test]
    #[should_panic]
    fn test_clockwise_construction_panics() {
        VoroConvexPolygon::new(dented_box().into_iter().rev().collect());
    }

    #[test]
    #[should_panic]
    fn test_concave_construction_panics() {
        let mut concave = dented_box();
        concave.insert(3, Point { x: 0.0, y: 0.0 });
        VoroConvexPolygon::new(concave);
    }

    #[test]
    fn test_is_inside() {
        let polygon = VoroConvexPolygon::new(dented_box()).unwrap();
        let origin = Point { x: 0.0, y: 0.0 };
        let outside_point = Point { x: 123.0, y: 456.0 };
        assert!(polygon.is_inside(&origin), "Origin inside default polygon");
        assert!(
            !polygon.is_inside(&outside_point),
            "{:?} ouside default polygon",
            outside_point
        );
    }

    #[test]
    fn test_which_edge() {
        let polygon = VoroConvexPolygon::new(dented_box()).unwrap();
        assert_eq!(polygon.which_edge(&Point { x: 0.5, y: 0.5 }), Ok(0));
        assert_eq!(polygon.which_edge(&Point { x: 1.0, y: -0.5 }), Ok(1));
        assert_eq!(polygon.which_edge(&Point { x: 0.0, y: -1.0 }), Ok(2));
        assert_eq!(polygon.which_edge(&Point { x: -1.0, y: 0.0 }), Ok(3));
        assert_eq!(polygon.which_edge(&Point { x: -0.5, y: 1.0 }), Ok(4));

        assert!(polygon.which_edge(&Point { x: 0.0, y: 0.0 }).is_err());
        assert!(polygon.which_edge(&Point { x: 1.0, y: 1.0 }).is_err());
    }

    #[test]
    fn test_intersect_line() {
        let polygon = VoroConvexPolygon::new(dented_box()).unwrap();
        assert_eq!(
            polygon.intersect_line(&Point { x: -0.5, y: 0.0 }, &Point { x: 0.5, y: 0.0 }),
            (
                Some(Point { x: 1.0, y: 0.0 }),
                Some(Point { x: -1.0, y: 0.0 })
            )
        );
        assert_eq!(
            polygon.intersect_line(&Point { x: -2.0, y: 0.0 }, &Point { x: 0.0, y: -2.0 }),
            (Some(Point { x: -1.0, y: -1.0 }), None)
        );
        assert_eq!(
            polygon.intersect_line(&Point { x: -3.0, y: 0.0 }, &Point { x: 0.0, y: -3.0 }),
            (None, None)
        );
        assert_eq!(
            polygon.intersect_line(&Point { x: -1.0, y: 2.0 }, &Point { x: 2.0, y: -1.0 }),
            (
                Some(Point { x: 0.0, y: 1.0 }),
                Some(Point { x: 1.0, y: 0.0 })
            )
        );
    }
}
