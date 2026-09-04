// Functionality for computing splines in 2D
// Aims: to provide functionality for computing the positions of
// interpolated points along arbitrary splines, with various properties,
// including Bezier, Hermite, and B-splines

use glam::Vec2;

type Row = Vec<Vec2>;
type Matrix = Vec<Row>;

pub struct SplineSegment {
    pub control_points: Vec<Vec2>,
    pub char_matrix: Matrix,
}

pub struct Spline {
    pub segments: Vec<SplineSegment>,
}
