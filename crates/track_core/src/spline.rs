// Functionality for computing splines in 2D
// Aims: to provide functionality for computing the positions of
// interpolated points along arbitrary splines, with various properties,
// including Bezier, Hermite, and B-splines

// {1, 0, 0, 0},
// {-3, 3, 0, 0},
// {3, -6, 3, 0},
// {-1, 3, -3, 1}

pub struct CubicSplineSegment {
    pub control_points: [glam::Vec2; 4],
    pub char_matrix: [glam::Vec4; 4],
    pub weighted_points: [glam::Vec2; 4],
}

impl CubicSplineSegment {
    pub fn new(control_points: [glam::Vec2; 4], char_matrix: [glam::Vec4; 4]) -> Self {
        let weighted_points = Self::make_weighted_points(&control_points, &char_matrix);
        Self {
            control_points,
            char_matrix,
            weighted_points,
        }
    }

    pub fn bezier(control_points: [glam::Vec2; 4]) -> Self {
        Self::new(control_points, Self::bezier_matrix())
    }

    fn bezier_matrix() -> [glam::Vec4; 4] {
        [
            glam::Vec4::new(1.0, 0.0, 0.0, 0.0),
            glam::Vec4::new(-3.0, 3.0, 0.0, 0.0),
            glam::Vec4::new(3.0, -6.0, 3.0, 0.0),
            glam::Vec4::new(-1.0, 3.0, -3.0, 1.0),
        ]
    }

    fn make_weighted_points(
        control_points: &[glam::Vec2; 4],
        char_matrix: &[glam::Vec4; 4],
    ) -> [glam::Vec2; 4] {
        char_matrix
            .each_ref()
            .map(|row| weighted_sum(row, control_points))
    }

    pub fn evaluate(&self, s: f32) -> glam::Vec2 {
        let s2 = s * s;
        let s3 = s2 * s;
        self.weighted_points[0]
            + self.weighted_points[1] * s
            + self.weighted_points[2] * s2
            + self.weighted_points[3] * s3
    }

    pub fn evaluate_range(&self, n: u32) -> Vec<glam::Vec2> {
        if n == 0 {
            return Vec::new();
        }
        if n == 1 {
            return vec![self.evaluate(0.0)];
        }
        (0..n)
            .map(|i| self.evaluate(i as f32 / (n - 1) as f32))
            .collect()
    }
}

pub struct Spline {
    pub segments: Vec<CubicSplineSegment>,
}

impl Default for Spline {
    fn default() -> Self {
        Self {
            segments: Vec::new(),
        }
    }
}

impl Spline {
    /// Builds contiguous cubic Beziers: first segment needs 4 points, each
    /// further segment reuses the previous end point and needs 3 new points.
    pub fn from_bezier_points(points: &[glam::Vec2]) -> Self {
        let mut segments = Vec::new();
        if points.len() < 4 {
            return Self { segments };
        }
        let segment_count = (points.len() - 1) / 3;
        for i in 0..segment_count {
            let base = i * 3;
            segments.push(CubicSplineSegment::bezier([
                points[base],
                points[base + 1],
                points[base + 2],
                points[base + 3],
            ]));
        }
        Self { segments }
    }

    pub fn evaluate_range(&self, samples_per_segment: u32) -> Vec<glam::Vec2> {
        let mut samples = Vec::new();
        for (i, segment) in self.segments.iter().enumerate() {
            let segment_samples = segment.evaluate_range(samples_per_segment);
            if i > 0 {
                samples.extend(segment_samples.into_iter().skip(1));
            } else {
                samples.extend(segment_samples);
            }
        }
        samples
    }
}

pub fn weighted_sum(w: &glam::Vec4, pts: &[glam::Vec2; 4]) -> glam::Vec2 {
    pts.iter()
        .zip(w.to_array())
        .fold(glam::Vec2::ZERO, |acc, (p, w)| acc + *p * w)
}
