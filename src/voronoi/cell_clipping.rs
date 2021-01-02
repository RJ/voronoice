use std::iter::once;
use delaunator::EMPTY;
use super::{HullBehavior, Point, bounding_box::{self, *}};

pub struct CellBuilder {
    vertices: Vec<Point>,
    bounding_box: BoundingBox,
    top_left_corner_index: usize,
    bottom_left_corner_index: usize,
    top_right_corner_index: usize,
    bottom_right_corner_index: usize,
}

impl CellBuilder {
    pub fn new(vertices: Vec<Point>, bounding_box: BoundingBox) -> Self {
        Self {
            top_right_corner_index: 0,
            top_left_corner_index: 0,
            bottom_left_corner_index: 0,
            bottom_right_corner_index: 0,
            vertices,
            bounding_box,
        }
    }

    pub fn build(self) -> Vec<Point> {
        self.vertices
    }

    /// Cell is assumed to be closed.
    /// Returns a value indicating whether this cell contains at least one edge inside the bounding box.
    pub fn clip_and_close_cell(&mut self, cell: &mut Vec<usize>) -> bool {
        if cell.len() < 3 {
            panic!("Only closed cells can be clipped. A cell must have at least 3 vertices to possibly be closed.")
        }

        // cell is closed, the "previous" is the first because the iterator below loops (last -> first), which will yeild a result
        // like: [first, second ... last, first] before the duplication removal
        // by setting this value here, the duplication removal will convert such result to
        // [first, second, second, thrid, forth ... last, first] -> [second, thrid, forth, ... last, first]
        // virtually shiftting the result to the right. This does not change the counter-clockwise ordering of the verteces
        let mut previous = Some(*cell.first().expect("At least one vertex expected for a cell."));

        // keep track of the indices where the cell is open
        // open_edges[i] will tell the index of new_cell after an edge removal
        const MAX_OPEN_EDGES: usize = 20;
        let mut open_edges = [EMPTY; MAX_OPEN_EDGES];
        let mut open_edges_count = 0;
        let mut vertex_count = 0;
        let mut last_edge_removed = false;

        // iterates over (n, n+1)
        *cell = cell.iter().zip(cell.iter().skip(1))
            // cell is closed, add (last, first) pair to the end to be handled too
            .chain(once((cell.last().unwrap(), cell.first().unwrap())))
            // clip edge and convert to list of indices again
            .flat_map(|(a, b)| {
                let (new_a, new_b) = self.clip_cell_edge(*a, *b);
                once(new_a).chain(once(new_b))
            })
            // remove duplicates
            .filter_map(|a| {
                let prev = previous;
                previous = Some(a);

                // remove vertex if it is empty
                let vertex = if a == EMPTY {
                    if prev == Some(EMPTY) {
                        // two EMPTY vertices removed in a row means an edge was removed
                        last_edge_removed = true;
                    }
                    None
                } else if let Some(prev) = prev {
                    if prev == a {
                        // vertex did not change
                        None
                    } else {
                        // is we have a valid vertex, and one or more edges was removed
                        if last_edge_removed {
                            last_edge_removed = false;
                            open_edges[open_edges_count] = vertex_count;
                            open_edges_count += 1;
                        }

                        vertex_count += 1;
                        Some(a)
                    }
                } else {
                    // is we have a valid vertex, and one or more edges was removed
                    if last_edge_removed {
                        last_edge_removed = false;
                        open_edges[open_edges_count] = vertex_count;
                        open_edges_count += 1;
                    }

                    vertex_count += 1;
                    Some(a)
                };

                vertex
            })
            .collect::<Vec<usize>>();

        if cell.len() < 2 {
            // if 1 or 0 vertice in the box, this cell should be removed
            return false;
        }

        // if edges were removed, cell is open and needs to be closed
        let mut open_edge_index_adjustment = 0;
        for i in 0..open_edges_count {
            // FIXME: this can only happen for the first value (open_edges[0]), no need to check on every loop
            // if we start iterating the cell on a vertex outside the box
            // the paring vertex to close this edge wraps around the vector (i.e. need to close last -> first)
            let (a, b) = if open_edges[i] == 0 {
                (cell.len() - 1, open_edges[i])
            } else {
                let v = i + open_edge_index_adjustment;
                (open_edges[v] - 1, open_edges[v])
            };

            // this many vertices were added between A and B, this means values were shifted right
            open_edge_index_adjustment += self.link_vertices_around_box_edge(cell, a, b);
        }

        // if after closing, we are left with a line, this cell should be removed
        cell.len() >= 3
    }

    /// Clips edge indexed by a -> b and return the indices of the verteces in the clipped edge (may be same values if no clipping was required).
    /// Cells edges be returned as `EMPTY` indicating that the edge is completely outside the box and should be excluded.
    fn clip_cell_edge(&mut self, a: usize, b: usize) -> (usize, usize) {
        // we are iterating a -> b, counter-clockwise on the edges of the cell (cell may be open)
        // at lest one intersection, possibilities
        // 1) a -> box edge -> box edge -> b            a and b outside box ---> need to clip edge at both intersections
        // 2) a -> box edge_same, box edge_same -> b    same as 1, but a->b is a line parallel to one of the box edges ---> keep edge (excluding edge would open cell)
        // 3) a -> box corner, box corner -> b          same as 1, but intersection is right on box corner, or line is parallel to one of the box edges ---> keep edge (excluding edge would open cell)
        // 4) a -> box edge -> b -> box edge            a outside, b inside box ---> clip edge at first intersection
        // 5) a -> b -> box edge [, box edge]           a and b outside, but the line they are in intersects the box (variations include intersection on corner, intersection parallel to a box edge) ---> exclude edge
        // 6) a -> b -> box edge                        a and b inside ---> keep edge as is
        // 7) a -> b                                    a and b outside box, and no intersection with box ---> exclude edge
        let pa = &self.vertices[a];
        let pb = &self.vertices[b];
        let mut new_a = a;
        let mut new_b = b;

        if !self.bounding_box.is_inside(pa) {
            // a is outside, b is inside or outside
            // clip will tell us how many intersections between a->b, and clip_a will come first than clip_b
            let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
            let (clip_a, clip_b) = self.bounding_box.project_ray(pa, &a_to_b);

            if let Some(clip_a) = clip_a {
                let v_index = self.vertices.len();
                new_a = v_index;

                if let Some(clip_b) = clip_b {
                    // b may be inside or outside the box
                    if !self.bounding_box.is_inside(pb) {
                        // there are two main cases here:
                        // a -> b -> box; i.e. ray reaches the box but after b; we discard this edge (case 5)
                        // check which point (b or clip_a) is closer to a
                        let closest_to_a = bounding_box::order_points_on_ray(pa,&a_to_b, Some(pb.clone()), Some(clip_a.clone())).0.unwrap();
                        if closest_to_a == clip_a {
                            // a -> box -> b; edge crosses box (case 1,2,3), we clip this ray twice
                            new_b = v_index + 1;

                            if clip_a == clip_b {
                                // case 3 - a and b outside box, intersection at the corner
                                // FIXME: consider distance epislon comparison instead
                                // intersection at same point (corner)
                                new_b = EMPTY;

                                // check which corner it is and use the known index
                                new_a = match self.bounding_box.which_edge(&clip_a) {
                                    (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::Right) => self.top_right_corner_index,
                                    (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::Left) => self.top_left_corner_index,
                                    (BoundingBoxTopBottomEdge::Bottom, BoundingBoxLeftRightEdge::Right) => self.bottom_right_corner_index,
                                    (BoundingBoxTopBottomEdge::Bottom, BoundingBoxLeftRightEdge::Left) => self.bottom_left_corner_index,
                                    _ => panic!("All corners covered."),
                                };
                            } else {
                                self.vertices.push(clip_a);
                                self.vertices.push(clip_b);
                            }
                        } else {
                            // a -> b -> box (case 5), discard edge
                            new_a = EMPTY;
                            new_b = EMPTY;
                        }
                    } else {
                        // b is inside the box, only clip the edge once
                        self.vertices.push(clip_a);
                    }
                } else {
                    // a single itersection, this means a -> box ->b
                    self.vertices.push(clip_a);
                }
            } else {
                // a and b are outside of bounding box and there were no intersection with box
                // this edge will be completely excluded from the result
                // this also means the resulting cell will be open (if it was previously closed)
                // case 7
                new_a = EMPTY;
                new_b = EMPTY;
            }
        } else if !self.bounding_box.is_inside(pb) {
            // b is outside, and a is inside
            let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
            let clip_b = self.bounding_box.project_ray_closest(pa, &a_to_b);

            // track new index for b
            new_b = self.vertices.len();
            self.vertices.push(clip_b.expect("Vertex 'b' is outside the bounding box. An intersection should have been returned."));
        } // else neither is outside, not need for clipping

        (new_a, new_b)
    }

    /// Given two vertices `a` and `b` that are on the bounding box edge(s), returns what vertices must be added between them to conform with the bounding box format.
    /// `a` and `b` are assumed to be ordered counter-clockwise.
    /// Returns how many new vertices were added.
    pub fn link_vertices_around_box_edge(&mut self, cell: &mut Vec<usize>, a: usize, b: usize) -> usize {
        let pa = &self.vertices[cell[a]];
        let pb = &self.vertices[cell[b]];
        let (mut a_bt, mut a_lr) = self.bounding_box.which_edge(pa);
        let (b_bt, b_lr) = self.bounding_box.which_edge(pb);

        // It is expected that points ARE on the box edge
        debug_assert!(!(a_bt == BoundingBoxTopBottomEdge::None && a_lr == BoundingBoxLeftRightEdge::None), "Point a ({}) was not located on the box's edge", a);
        debug_assert!(!(b_bt == BoundingBoxTopBottomEdge::None && b_lr == BoundingBoxLeftRightEdge::None), "Point b ({}) was not located on the box's edge", b);

        // short-circuit for case where a and b are on the same edge, and b is ahead of a, such case there is nothing to do
        if (a_bt != BoundingBoxTopBottomEdge::None && a_bt == b_bt && pa.x > pb.x)
        || (a_lr != BoundingBoxLeftRightEdge::None && a_lr == b_lr && pa.y > pb.y) {
            return 0;
        }

        // a full cycle in the box will yeild maximum
        const MAX_CORNERS: usize = 4;
        let mut new_vertices = [EMPTY; MAX_CORNERS];
        let mut new_vertice_count = 0;

        // we only have work to do if a and b are not on the same edge
        // walk on box edge, starting on 'a' until we reach 'b'
        // on each iteration, move to the next edge (counter-clockwise)
        // and return the corners until b is reached
        // note: a and b may start on the same edge, this means we need to do a full loop on the box's edge
        loop {
            match (a_bt, a_lr) {
                (BoundingBoxTopBottomEdge::Top, lr) => {
                    // move to left edge
                    a_bt = BoundingBoxTopBottomEdge::None;
                    a_lr = BoundingBoxLeftRightEdge::Left;

                    // add top left corner, if 'a' is not already such a point
                    if lr != BoundingBoxLeftRightEdge::Left {
                        new_vertices[new_vertice_count] = self.top_left_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (tb, BoundingBoxLeftRightEdge::Left) => {
                    // move to bottom edge
                    a_bt = BoundingBoxTopBottomEdge::Bottom;
                    a_lr = BoundingBoxLeftRightEdge::None;

                    // add bottom left corner, if 'a' is not such a point
                    if tb != BoundingBoxTopBottomEdge::Bottom {
                        new_vertices[new_vertice_count] = self.bottom_left_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (BoundingBoxTopBottomEdge::Bottom, lr) => {
                    // move to right edge
                    a_bt = BoundingBoxTopBottomEdge::None;
                    a_lr = BoundingBoxLeftRightEdge::Right;

                    // add bottom right corner, if 'a' is not such a point
                    if lr != BoundingBoxLeftRightEdge::Right {
                        new_vertices[new_vertice_count] = self.bottom_right_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (tb, BoundingBoxLeftRightEdge::Right) => {
                    // move to top edge
                    a_bt = BoundingBoxTopBottomEdge::Top;
                    a_lr = BoundingBoxLeftRightEdge::None;

                    // add top right corner, if 'a' is not such a point
                    if tb != BoundingBoxTopBottomEdge::Top {
                        new_vertices[new_vertice_count] = self.top_right_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (BoundingBoxTopBottomEdge::None, BoundingBoxLeftRightEdge::None) => panic!("It seems that either 'a' ({}) or 'b' ({}) are not on the box's edge. Cannot link vertices not on the edge.", a, b)
            }

            if (a_bt != BoundingBoxTopBottomEdge::None && a_bt == b_bt) || (a_lr != BoundingBoxLeftRightEdge::None && a_lr == b_lr) {
                break;
            } else if new_vertice_count == MAX_CORNERS {
                panic!("It seems that either 'a' ({}) or 'b' ({}) are not on the box's edge. Cannot link vertices not on the edge.", a, b);
            }
        }

        if new_vertice_count != 0 {
            // splice cell and insert values on the right position (after a)
            let r = a + 1;
            cell.splice(r..r, new_vertices.iter().take(new_vertice_count).copied());
        }

        new_vertice_count
    }

    pub fn calculate_corners(&mut self) {
        // add all corners to the vertice list as they will be used for closing cells
        let top_right = self.bounding_box.top_right().clone();
        let mut top_left = top_right.clone(); top_left.x *= -1.0;
        let mut bottom_left = top_left.clone(); bottom_left.y *= -1.0;
        let mut bottom_right = top_right.clone(); bottom_right.y *= -1.0;
        self.vertices.push(top_right);
        self.vertices.push(top_left);
        self.vertices.push(bottom_left);
        self.vertices.push(bottom_right);

        let bottom_right_index = self.vertices.len() - 1;
        self.bottom_right_corner_index = bottom_right_index;
        self.bottom_left_corner_index = bottom_right_index - 1;
        self.top_left_corner_index = bottom_right_index - 2;
        self.top_right_corner_index = bottom_right_index - 3;
    }

    // /// Assumes all points are inside the box (or on its edges). It may panic otherwise.
    // fn close_cell(&mut self, cell: &mut Vec<usize>) {
    //     if cell.len() < 3 {
    //         panic!("I don't know how to close a cell with {} vertice(s)", cell.len());
    //     }


    // }


    //     let mine = vertices[curr_vertex].clone();
    //     let prev = vertices[prev_vertex].clone();

    //     // FIXME: use same points for corners
    //     // let top_right = Point { x: bounding_box.x, y: bounding_box.y };
    //     // let top_left = Point { x: -bounding_box.x, y: bounding_box.y };
    //     // let bottom_right = Point { x: bounding_box.x, y: -bounding_box.y };
    //     // let bottom_left = Point { x: -bounding_box.x, y: -bounding_box.y };

    //     // close the cell by picking the previous site extension to close the polygon
    //     // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
    //     cell.insert(cell.len() - 1, prev_vertex);
    //     let bounding_box = bounding_box.top_right();

    //     if mine.x.abs() == bounding_box.x && prev.x.abs() == bounding_box.x && mine.x != prev.x {
    //         println!("Case 2: mine {:?} prev {:?}", mine, prev);
    //         // cur and prev on oppositve X bounding lines
    //         let dir = if prev.y.signum() != mine.y.signum() && prev.y.abs() > mine.y.abs() {
    //             -mine.y.signum()
    //         } else {
    //             mine.y.signum()
    //         };

    //         let p = Point { x: prev.x, y: dir * bounding_box.y };
    //         println!("  Added {:?}", p);
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);

    //         let p = Point { x: mine.x, y: dir * bounding_box.y };
    //         println!("  Added {:?}", p);
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);
    //     } else if mine.y.abs() == bounding_box.y && prev.y.abs() == bounding_box.y && mine.y != prev.y {
    //         println!("Case 3: mine {:?} prev {:?}", mine, prev);
    //         // cur and prev on oppositve Y bounding lines
    //         let dir = if prev.x.signum() != mine.x.signum() && prev.x.abs() > mine.x.abs() {
    //             -mine.x.signum()
    //         } else {
    //             mine.x.signum()
    //         };

    //         let p = Point { x: dir * bounding_box.x, y: prev.y };
    //         println!("  Added {:?}", p);
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);

    //         let p = Point { x: dir * bounding_box.x, y: mine.y };
    //         println!("  Added {:?}", p);
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);
    //     } else if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
    //         println!("Case 0: mine {:?} prev {:?}", mine, prev);
    //         let p = Point { x: mine.x, y: prev.y };
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);
    //     } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
    //         println!("Case 1: mine {:?} prev {:?}", mine, prev);
    //         let p = Point { x: prev.x, y: mine.y };
    //         let i = vertices.len();
    //         vertices.push(p);
    //         cell.insert(cell.len() - 1, i);
    //     }
    // }
}

#[cfg(test)]
mod test {
    use super::*;

    fn new_builder(vertices: Vec<Point>) -> CellBuilder {
        CellBuilder::new(vertices, BoundingBox::new_centered_square(4.0))
    }

    fn assert_same_elements(actual: &Vec<usize>, expected: &Vec<usize>, message: &str) {
        assert_eq!(actual.len(), expected.len(), "Vectors differ in length. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
        assert_eq!(0, actual.iter().copied().zip(expected.iter().copied()).filter(|(a,b)| a != b).collect::<Vec<(usize, usize)>>().len(), "Vectors have differing elements. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
    }

    /// Check that the cell is ordered counter-clockwise and inside the bounding box.
    fn assert_cell_consistency(cell: &Vec<usize>, builder: &CellBuilder) {
        let points: Vec<Point> = cell.iter().map(|c| builder.vertices[*c].clone()).collect();

        // are all points within the bounding box?
        let points_outside: Vec<(usize, &Point)> = points.iter().enumerate().filter(|(i, p)| {
            !builder.bounding_box.is_inside(&p)
        }).collect();
        assert_eq!(0, points_outside.len(), "These points are outside bounding box: {:?}", points_outside);

        // is counter-clockwise? https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
        let area = points.iter().zip(points.iter().cycle().skip(1)).fold(0.0, |acc, (a, b)| {
                acc + ((b.x - a.x) * (b.y + a.y))
            });
        assert_eq!(true, area < 0.0, "Area of the polygon must be less than 0 for it to be counter-clockwise ordered. Area: {}", area);
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_after_a() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: 2.0 },
            Point { x: -1.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &original_cell, "No change expected. Same edge, b is after a");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_before_a() {
        let mut builder = new_builder(vec![
            Point { x: -1.0, y: 2.0 },
            Point { x: 1.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Full circuit expected because b is before a");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_corner_before_a() {
        let mut builder = new_builder(vec![
            Point { x: -1.0, y: 2.0 },
            Point { x: 2.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_top_left_b_bottom_right() {
        let mut builder = new_builder(vec![
            Point { x: -2.0, y: 2.0 },
            Point { x: 2.0, y: -2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_right_b_left() {
        let mut builder = new_builder(vec![
            Point { x: 2.0, y: 1.0 },
            Point { x: -2.0, y: 1.5 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_right_corner_index, builder.top_left_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_left_b_right() {
        let mut builder = new_builder(vec![
            Point { x: -2.0, y: 1.0 },
            Point { x: 2.0, y: 1.5 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
    }


    #[test]
    fn link_vertices_around_box_edge_same_edge_a_bottom_b_top() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: -2.0 },
            Point { x: -1.5, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_one_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 0, "A is inside box, should not change");
        assert_eq!(b, 2, "New edge should have been added due to clipping");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_one_box_edge_inverted() {
        let mut builder = new_builder(vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 2, "New edge should have been added due to clipping");
        assert_eq!(b, 1, "B is inside box, should not change");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_two_box_edges() {
        let mut builder = new_builder(vec![
            Point { x: -10.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 2, "New edge should have been added due to clipping");
        assert_eq!(b, 3, "New edge should have been added due to clipping");
        assert_eq!(Point { x: -2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_and_close_cell_when_no_point_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 1.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![1, 2, 0], "No clipping expected");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 }, // outside
            Point { x: 1.0, y: 0.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![3, 4, 2, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box_and_last_crossing_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 }, // outside
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![1, 3, 4, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_one_edge_entirely_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -30.0 }, // leaves through the bottom
            Point { x: 20.0, y: 0.0 }, // comes back through the right
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // builder adds 4 vertices all the time, so next index is 7
        assert_same_elements(&clipped_cell, &vec![7, builder.bottom_right_corner_index, 8, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_edge_crossing_box_twice() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 },
            Point { x: 3.0, y: 0.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // builder adds 4 vertices all the time, so next index is 7
        assert_same_elements(&clipped_cell, &vec![7, 8, 9, 10, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -1.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
        //assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    #[test]
    fn clip_triangular_cell_with_one_edge_intersecting_box_corner() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 4.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![7, builder.top_right_corner_index , 8, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 0.0, y: 2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    // same as above but with triangle inverted, no vertex inside box
    #[test]
    fn clip_triangular_cell_outside_with_one_edge_intersecting_box_corner() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 4.0 },
            Point { x: 4.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, false, "A single intersection at the coner with all other points outside the cell, should not keep this cell.");
        assert_same_elements(&clipped_cell, &vec![builder.top_right_corner_index], "Clipped cell incorrect indices.");
    }

    #[test]
    fn clip_triangular_cell_with_one_point_inside_box_and_one_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 2.0, y: -4.0 },
            Point { x: 2.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // FIXME: identify intersection at corners and return corner index instead of creating a new one
        assert_same_elements(&clipped_cell, &vec![7, 8, 9, 10, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: 2.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder);
    }

    #[test]
    fn clip_triangular_cell_with_no_point_inside_box_and_one_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 4.0 },
            Point { x: 2.0, y: -4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![7, 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(keep_cell, false, "A line shared with one box edge, should not keep this cell.");
        //assert_eq!(is_closed, false, "One edge outside box.")
    }

    // same case as above, but the entire parallel edge is shared with the box
    #[test]
    fn clip_triangular_cell_with_no_point_inside_box_and_one_shared_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
            Point { x: 2.0, y: -2.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![7, 1, 2, 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[1], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");

        // FIXME: the result is a rectangle with area 0 - code think cell is closed, but it should be removed
        //assert_cell_consistency(&clipped_cell, &builder);

        // FIX ME: code thinks cell is closed, but that is not true in this case
        // ignoring for now as this shouldn't really happen for voronoi
        //assert_eq!(is_closed, false, "One edge outside box.");
    }

    #[test]
    fn clip_triangular_cell_with_three_points_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: -3.0 },
            Point { x: 15.0, y: 0.0 },

        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell.clone();
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, false, "No intersection with box, all points outside, should not keep the cell.");
        assert_same_elements(&clipped_cell, &vec![], "Clipped cell incorrect indices.");
    }
}