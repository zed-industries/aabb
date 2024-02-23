use std::{cmp, fmt, iter};

#[derive(Debug)]
pub struct BoundsTree<T> {
    root: Option<usize>,
    nodes: Vec<Node<T>>,
    stack: Vec<usize>,
}

impl<T: Clone> BoundsTree<T> {
    pub fn new() -> Self {
        BoundsTree {
            root: None,
            nodes: Vec::new(),
            stack: Vec::new(),
        }
    }

    pub fn insert(&mut self, new_bounds: Bounds, data: T) -> u32 {
        // If the tree is empty, make the root the new leaf.
        if self.root.is_none() {
            let new_node = self.push_leaf(new_bounds, data, 1);
            self.root = Some(new_node);
            return 1;
        }

        // Search for the best place to add the new leaf based on heuristics.
        let mut max_intersecting_ordering = 0;
        let mut index = self.root.unwrap();
        while let Node::Internal {
            left,
            right,
            bounds: node_bounds,
            ..
        } = self.node_mut(index)
        {
            let left = *left;
            let right = *right;
            *node_bounds = node_bounds.merge(new_bounds);
            self.stack.push(index);

            // Descend to the best-fit child, based on which one would increase
            // the surface area the least. This attempts to keep the tree balanced
            // in terms of surface area. If there is an intersection with the other child,
            // add its keys to the intersections vector.
            let left_cost = new_bounds.merge(self.node(left).bounds()).half_perimeter();
            let right_cost = new_bounds.merge(self.node(right).bounds()).half_perimeter();
            if left_cost < right_cost {
                max_intersecting_ordering =
                    self.collect_max_ordering(right, new_bounds, max_intersecting_ordering);
                index = left;
            } else {
                max_intersecting_ordering =
                    self.collect_max_ordering(left, new_bounds, max_intersecting_ordering);
                index = right;
            }
        }

        // We've found a leaf ('index' now refers to a leaf node).
        // We'll insert a new parent node above the leaf and attach our new leaf to it.
        let sibling = index;

        // Check for collision with the located leaf node
        let Node::Leaf {
            bounds: sibling_bounds,
            order: sibling_ordering,
            ..
        } = self.node(index)
        else {
            unreachable!();
        };
        if sibling_bounds.intersects(new_bounds) {
            max_intersecting_ordering = cmp::max(max_intersecting_ordering, *sibling_ordering);
        }

        let ordering = max_intersecting_ordering + 1;
        let new_node = self.push_leaf(new_bounds, data, ordering);
        let new_parent = self.push_internal(sibling, new_node);

        // If there was an old parent, we need to update its children indices.
        if let Some(old_parent) = self.stack.last().copied() {
            let Node::Internal { left, right, .. } = self.node_mut(old_parent) else {
                unreachable!();
            };

            if *left == sibling {
                *left = new_parent;
            } else {
                *right = new_parent;
            }
        } else {
            // If the old parent was the root, the new parent is the new root.
            self.root = Some(new_parent);
        }

        for node_index in self.stack.drain(..) {
            let Node::Internal { max_ordering, .. } = &mut self.nodes[node_index] else {
                unreachable!()
            };
            *max_ordering = cmp::max(*max_ordering, ordering);
        }

        ordering
    }

    pub fn iter(&self) -> impl Iterator<Item = Primitive<&T>> {
        let mut stack = Vec::new();
        stack.extend(self.root);
        iter::from_fn(move || {
            while let Some(node_ix) = stack.pop() {
                match self.node(node_ix) {
                    Node::Leaf {
                        bounds,
                        data,
                        order,
                        ..
                    } => {
                        return Some(Primitive {
                            bounds: *bounds,
                            data,
                            order: *order,
                        });
                    }
                    Node::Internal { left, right, .. } => {
                        stack.push(*left);
                        stack.push(*right);
                    }
                }
            }
            None
        })
    }

    fn collect_max_ordering(&self, index: usize, bounds: Bounds, max_ordering: u32) -> u32 {
        match self.node(index) {
            Node::Leaf {
                bounds: node_bounds,
                order: ordering,
                ..
            } => {
                if bounds.intersects(*node_bounds) {
                    cmp::max(*ordering, max_ordering)
                } else {
                    max_ordering
                }
            }
            Node::Internal {
                left,
                right,
                bounds: node_bounds,
                max_ordering: node_max_ordering,
                ..
            } => {
                if bounds.intersects(*node_bounds) && max_ordering < *node_max_ordering {
                    let left_max_ordering = self.collect_max_ordering(*left, bounds, max_ordering);
                    let right_max_ordering =
                        self.collect_max_ordering(*right, bounds, max_ordering);
                    cmp::max(left_max_ordering, right_max_ordering)
                } else {
                    max_ordering
                }
            }
        }
    }

    fn push_leaf(&mut self, bounds: Bounds, data: T, order: u32) -> usize {
        self.nodes.push(Node::Leaf {
            bounds,
            data,
            order,
        });
        self.nodes.len() - 1
    }

    fn push_internal(&mut self, left: usize, right: usize) -> usize {
        let left_node = self.node(left);
        let right_node = self.node(right);
        let new_bounds = left_node.bounds().merge(right_node.bounds());
        let max_ordering = cmp::max(left_node.max_ordering(), right_node.max_ordering());
        self.nodes.push(Node::Internal {
            bounds: new_bounds,
            left,
            right,
            max_ordering,
        });
        self.nodes.len() - 1
    }

    #[inline(always)]
    fn node(&self, index: usize) -> &Node<T> {
        &self.nodes[index]
    }

    #[inline(always)]
    fn node_mut(&mut self, index: usize) -> &mut Node<T> {
        &mut self.nodes[index]
    }
}

#[derive(Default, Debug, Clone, PartialEq)]
pub struct Primitive<T> {
    data: T,
    bounds: Bounds,
    order: u32,
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: Point,
    pub max: Point,
}

impl Bounds {
    fn merge(self, other: Bounds) -> Bounds {
        Bounds {
            min: Point {
                x: self.min.x.min(other.min.x),
                y: self.min.y.min(other.min.y),
            },
            max: Point {
                x: self.max.x.max(other.max.x),
                y: self.max.y.max(other.max.y),
            },
        }
    }

    fn intersects(self, other: Bounds) -> bool {
        !(self.min.x >= other.max.x
            || self.max.x <= other.min.x
            || self.min.y >= other.max.y
            || self.max.y <= other.min.y)
    }

    fn half_perimeter(self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        width + height
    }
}

#[derive(Debug)]
enum Node<T> {
    Leaf {
        bounds: Bounds,
        data: T,
        order: u32,
    },
    Internal {
        left: usize,
        right: usize,
        bounds: Bounds,
        max_ordering: u32,
    },
}

impl<T> Node<T> {
    fn bounds(&self) -> Bounds {
        match self {
            Node::Leaf { bounds, .. } => *bounds,
            Node::Internal { bounds, .. } => *bounds,
        }
    }

    fn max_ordering(&self) -> u32 {
        match self {
            Node::Leaf {
                order: ordering, ..
            } => *ordering,
            Node::Internal { max_ordering, .. } => *max_ordering,
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(x: {:.2}, y: {:.2})", self.x, self.y)
    }
}

#[cfg(test)]
mod tests {
    use std::{fs, path::Path};

    use super::*;
    use rand::{Rng, SeedableRng};

    #[test]
    fn test_bounds_insertion_with_two_bounds() {
        let mut tree = BoundsTree::new();
        let bounds1 = Bounds {
            min: Point { x: 0.0, y: 0.0 },
            max: Point { x: 10.0, y: 10.0 },
        };
        let bounds2 = Bounds {
            min: Point { x: 5.0, y: 5.0 },
            max: Point { x: 15.0, y: 15.0 },
        };

        // Insert the first AABB.
        assert_eq!(tree.insert(bounds1, "bounds1".to_string()), 1);

        // Insert the second AABB, which overlaps with the first.
        assert_eq!(tree.insert(bounds2, "bounds2".to_string()), 2);
    }

    #[test]
    fn test_adjacent_bounds() {
        let mut tree = BoundsTree::new();
        let bounds1 = Bounds {
            min: Point { x: 0.0, y: 0.0 },
            max: Point { x: 10.0, y: 10.0 },
        };
        let bounds2 = Bounds {
            min: Point { x: 10.0, y: 0.0 },
            max: Point { x: 20.0, y: 10.0 },
        };

        // Insert the first bounds.
        assert_eq!(tree.insert(bounds1, "bounds1"), 1);

        // Insert the second bounds, which is adjacent to the first but not overlapping.
        assert_eq!(tree.insert(bounds2, "bounds2"), 1);
    }

    #[test]
    fn test_random_iterations() {
        let max_bounds = 100;

        let mut actual_intersections: Vec<usize> = Vec::new();
        for seed in 1..=1000 {
            // let seed = 44;
            let debug = false;
            if debug {
                let svg_path = Path::new("./svg");
                if svg_path.exists() {
                    fs::remove_dir_all("./svg").unwrap();
                }
                fs::create_dir_all("./svg").unwrap();
            }

            dbg!(seed);

            let mut tree = BoundsTree::new();
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed as u64);
            let mut expected_quads: Vec<Primitive<usize>> = Vec::new();

            let mut insert_time = std::time::Duration::ZERO;

            // Insert a random number of random AABBs into the tree.
            let num_bounds = rng.gen_range(1..=max_bounds);
            for quad_id in 0..num_bounds {
                let min_x: f32 = rng.gen_range(-100.0..100.0);
                let min_y: f32 = rng.gen_range(-100.0..100.0);
                let max_x: f32 = rng.gen_range(min_x..min_x + 50.0);
                let max_y: f32 = rng.gen_range(min_y..min_y + 50.0);
                let bounds = Bounds {
                    min: Point { x: min_x, y: min_y },
                    max: Point { x: max_x, y: max_y },
                };

                let expected_ordering = expected_quads
                    .iter()
                    .filter_map(|quad| quad.bounds.intersects(bounds).then_some(quad.order))
                    .max()
                    .unwrap_or(0)
                    + 1;
                expected_quads.push(Primitive {
                    bounds,
                    data: quad_id,
                    order: expected_ordering,
                });
                if debug {
                    println!("inserting {} with AABB: {:?}", quad_id, bounds);
                    draw_bounds(
                        format!("./svg/expected_bounds_after_{}.svg", quad_id),
                        &expected_quads,
                    );
                }

                // Insert the AABB into the tree and collect intersections.
                actual_intersections.clear();
                let t0 = std::time::Instant::now();
                let actual_ordering = tree.insert(bounds, quad_id);
                insert_time += t0.elapsed();
                assert_eq!(actual_ordering, expected_ordering);

                if debug {
                    draw_bounds_tree(format!("./svg/bounds_tree_after_{}.svg", quad_id), &tree);
                }

                // Verify the tree contains all the AABBs.
                let mut actual_quads = tree
                    .iter()
                    .map(|quad| Primitive {
                        bounds: quad.bounds,
                        data: *quad.data,
                        order: quad.order,
                    })
                    .collect::<Vec<_>>();
                actual_quads.sort_by_key(|quad| quad.data);
                expected_quads.sort_by_key(|quad| quad.data);
                assert_eq!(actual_quads, expected_quads);
            }

            dbg!(insert_time);
        }
    }

    fn draw_bounds(svg_path: impl AsRef<Path>, bounds: &[Primitive<usize>]) {
        let mut svg_content = String::from(
            r#"<svg xmlns="http://www.w3.org/2000/svg" version="1.1" viewBox="-100 -100 200 200" style="border:1px solid black;">"#,
        );

        for quad in bounds {
            svg_content.push_str(&format!(
                r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:none;stroke:black;stroke-width:1" />"#,
                quad.bounds.min.x,
                quad.bounds.min.y,
                quad.bounds.max.x - quad.bounds.min.x,
                quad.bounds.max.y - quad.bounds.min.y
            ));
            svg_content.push_str(&format!(
                r#"<text x="{}" y="{}" font-size="3" text-anchor="middle" alignment-baseline="central">{}</text>"#,
                (quad.bounds.min.x + quad.bounds.max.x) / 2.0,
                (quad.bounds.min.y + quad.bounds.max.y) / 2.0,
                quad.data
            ));
        }

        svg_content.push_str("</svg>");
        fs::write(svg_path, &svg_content).unwrap();
    }

    fn draw_bounds_tree(svg_path: impl AsRef<Path>, tree: &BoundsTree<usize>) {
        let mut svg_content = String::from(
            r#"<svg xmlns="http://www.w3.org/2000/svg" version="1.1" viewBox="-100 -100 200 200" style="border:1px solid black;">"#,
        );

        fn draw_node<T: fmt::Debug>(svg_content: &mut String, nodes: &[Node<T>], index: usize) {
            match &nodes[index] {
                Node::Internal {
                    bounds,
                    left,
                    right,
                    ..
                } => {
                    svg_content.push_str(&format!(
                        r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:rgba({},{},{},0.5);stroke:rgba({},{},{},1);stroke-width:1" />"#,
                        bounds.min.x,
                        bounds.min.y,
                        bounds.max.x - bounds.min.x,
                        bounds.max.y - bounds.min.y,
                        (index * 50) % 255, // Red component
                        (index * 120) % 255, // Green component
                        (index * 180) % 255, // Blue component
                        (index * 50) % 255, // Red stroke
                        (index * 120) % 255, // Green stroke
                        (index * 180) % 255  // Blue stroke
                    ));
                    draw_node(svg_content, nodes, *left);
                    draw_node(svg_content, nodes, *right);
                }
                Node::Leaf { bounds, data, .. } => {
                    svg_content.push_str(&format!(
                        r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:none;stroke:black;stroke-width:1" />"#,
                        bounds.min.x,
                        bounds.min.y,
                        bounds.max.x - bounds.min.x,
                        bounds.max.y - bounds.min.y
                    ));
                    svg_content.push_str(&format!(
                        r#"<text x="{}" y="{}" font-size="3" text-anchor="middle" alignment-baseline="central">{:?}</text>"#,
                        (bounds.min.x + bounds.max.x) / 2.0,
                        (bounds.min.y + bounds.max.y) / 2.0,
                        data
                    ));
                }
            }
        }

        if let Some(root) = tree.root {
            draw_node(&mut svg_content, &tree.nodes, root);
        }

        svg_content.push_str("</svg>");
        fs::write(svg_path, &svg_content).unwrap();
    }
}
