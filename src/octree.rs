use std::ops::{Index, IndexMut};

use bevy::{
    math::{U8Vec3, UVec3},
    platform::collections::HashMap,
};

/// Tracks the `level` we're on and the `octant` of `pos` in the level.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct LevelCursor<'a> {
    level: u32,
    pos: &'a UVec3,
    octant: u32,
}

impl<'a> LevelCursor<'a> {
    const fn root(depth: u32, pos: &'a UVec3) -> Self {
        LevelCursor {
            // The octant of `pos` in the root level is always 0.
            octant: 0,
            level: depth,
            pos,
        }
    }

    fn key(&self) -> Key {
        Key::new(*self.pos, self.level)
    }

    #[track_caller]
    fn descend(&mut self) {
        self.level -= 1;
        self.octant = get_octant(*self.pos, self.level);
    }

    #[track_caller]
    fn descended(mut self) -> Self {
        self.descend();
        self
    }

    #[track_caller]
    fn ascend(&mut self) {
        self.level += 1;
        self.octant = get_octant(*self.pos, self.level);
    }

    #[track_caller]
    fn ascended(mut self) -> Self {
        self.ascend();
        self
    }
}

/// Tracks the `index` and `tree_len` of the level we're on.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct TreeCursor {
    index: u32,
    tree_len: u32,
}

impl TreeCursor {
    const fn root(depth: u32) -> Self {
        TreeCursor {
            index: 0,
            tree_len: tree_len(depth),
        }
    }

    #[track_caller]
    fn descend(&mut self, child: &LevelCursor) {
        self.tree_len = (self.tree_len - 1) / 8;
        self.index += 1 + child.octant * self.tree_len;
    }

    #[track_caller]
    fn descended(mut self, child: &LevelCursor) -> Self {
        self.descend(child);
        self
    }

    #[track_caller]
    fn ascend(&mut self, this: &LevelCursor) {
        self.index -= 1 + this.octant * self.tree_len;
        self.tree_len = self.tree_len * 8 + 1;
    }

    #[track_caller]
    fn ascended(mut self, this: &LevelCursor) -> Self {
        self.ascend(this);
        self
    }
}

#[derive(Clone, Debug)]
struct CompleteByteOctree {
    vec: Vec<u8>,
}

impl CompleteByteOctree {
    fn new(depth: u32, fill: u8) -> Self {
        let len = tree_len(depth) as usize;
        let vec = vec![fill; len];
        Self { vec }
    }

    #[track_caller]
    fn get_child(&self, cursor: TreeCursor, child: LevelCursor) -> bool {
        get_child(&self[cursor], child.octant)
    }

    #[track_caller]
    fn set_child(&mut self, cursor: TreeCursor, child: LevelCursor, value: bool) {
        set_child(&mut self[cursor], child.octant, value);
    }
}

impl Index<TreeCursor> for CompleteByteOctree {
    type Output = u8;

    fn index(&self, cursor: TreeCursor) -> &Self::Output {
        &self.vec[cursor.index as usize]
    }
}

impl IndexMut<TreeCursor> for CompleteByteOctree {
    fn index_mut(&mut self, cursor: TreeCursor) -> &mut Self::Output {
        &mut self.vec[cursor.index as usize]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct Key {
    pos: U8Vec3,
    level: u8,
}

impl Key {
    fn new(pos: UVec3, level: u32) -> Self {
        let level = level as u8;
        let pos = (pos >> level).as_u8vec3();
        Self { level, pos }
    }

    fn with_octant(mut self, octant: u32) -> Self {
        let x = octant & 1;
        let y = (octant >> 1) & 1;
        let z = (octant >> 2) & 1;
        self.pos += UVec3::new(x, y, z).as_u8vec3();
        self
    }
}

#[derive(Clone, Debug)]
struct OctreeInner<T> {
    values: HashMap<Key, T>,
    children_have_children: CompleteByteOctree,
    children_inherit_this: CompleteByteOctree,
}

impl<T> OctreeInner<T> {
    fn new(depth: u32) -> Self {
        Self {
            values: HashMap::new(),
            children_have_children: CompleteByteOctree::new(depth - 2, 0x00),
            children_inherit_this: CompleteByteOctree::new(depth - 1, 0xFF),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Octree<T, const D: u32 = 5> {
    root: T,
    inner: Option<OctreeInner<T>>,
}

impl<T, const D: u32> Octree<T, D>
where
    T: Clone + PartialEq,
{
    pub const fn new(root: T) -> Self {
        // OctreeInner panics for D < 2.
        // Technically D should be able to equal 0 and 1 but I'm too lazy to handle it.
        assert!(D > 1);
        assert!(D <= 8);
        Self { root, inner: None }
    }

    const fn have_root() -> TreeCursor {
        TreeCursor::root(D - 2)
    }

    const fn inherit_root() -> TreeCursor {
        TreeCursor::root(D - 1)
    }

    const fn level_root(pos: &UVec3) -> LevelCursor {
        LevelCursor::root(D, pos)
    }

    fn scan_have(
        p_have: &mut TreeCursor,
        p_inherit: &mut TreeCursor,
        n_level: &mut LevelCursor,
        have: &CompleteByteOctree,
    ) {
        while n_level.level != 0 {
            if !have.get_child(*p_have, *n_level) {
                break;
            }

            p_have.descend(&n_level);
            p_inherit.descend(&n_level);
            n_level.descend();
        }
    }

    fn scan_inheritance<'a>(
        p_inherit: &mut TreeCursor,
        n_level: &mut LevelCursor,
        inherit: &CompleteByteOctree,
        values: &'a HashMap<Key, T>,
        root: &'a T,
    ) -> &'a T {
        let mut value = root;
        // stop before the root (level = D) since the root cannot inherit
        while n_level.level != D - 1 {
            if !inherit.get_child(*p_inherit, *n_level) {
                value = values.get(&n_level.key()).unwrap();
                break;
            }

            n_level.ascend();
            p_inherit.ascend(&n_level);
        }
        value
    }

    pub fn get(&self, pos: UVec3) -> &T {
        let Some(OctreeInner {
            values,
            children_have_children: have,
            children_inherit_this: inherit,
        }) = &self.inner
        else {
            return &self.root;
        };

        let mut p_have = Self::have_root();
        let mut p_inherit = Self::inherit_root();
        let mut n_level = Self::level_root(&pos).descended();

        Self::scan_have(&mut p_have, &mut p_inherit, &mut n_level, have);
        Self::scan_inheritance(&mut p_inherit, &mut n_level, inherit, values, &self.root)

        // PSEUDO CODE
        //
        // walk inner.splits towards pos until we find a split node or hit level 0
        // now we know the position and the level to look at
        //
        // walk inner.inherits backwards from the level we ended at until we find a non-inherited node
        // if we hit the root return the root, it's NOT in the map because I expect many nodes to inherit the
        // root and don't want to be forced to hash a constant for a value that always exists
        //
        // this node must exist in the map so we unwrap it and return it
    }

    pub fn set(&mut self, pos: UVec3, value: T) {
        let OctreeInner {
            values,
            children_have_children: have,
            children_inherit_this: inherit,
        } = match &mut self.inner {
            Some(inner) => inner,
            None => {
                if self.root == value {
                    return;
                }
                self.inner.insert(OctreeInner::new(D))
            }
        };

        let base_p_have;
        let base_p_inherit;
        let base_n_level;
        {
            let mut p_have = Self::have_root();
            let mut p_inherit = Self::inherit_root();
            let mut n_level = Self::level_root(&pos).descended();
            Self::scan_have(&mut p_have, &mut p_inherit, &mut n_level, have);

            base_p_have = p_have;
            base_p_inherit = p_inherit;
            base_n_level = n_level;

            let old_value =
                Self::scan_inheritance(&mut p_inherit, &mut n_level, inherit, values, &self.root);
            if *old_value == value {
                return;
            }
        }

        if base_n_level.level == 0 {
            // sole inheritor (change inherited value)
            {
                let mut p_inherit = base_p_inherit;
                let mut n_level = base_n_level;

                let mut once = false;

                loop {
                    let sole_inheritor = inherit[p_inherit] == 1 << n_level.octant;
                    if !sole_inheritor {
                        break;
                    }

                    for octant in 0..8 {
                        if octant == n_level.octant {
                            continue;
                        }

                        let key = n_level.key().with_octant(octant);
                        let other_value = values.get(&key).unwrap();
                        if *other_value == value {
                            values.remove(&key);
                            inherit.set_child(p_inherit, n_level, true);
                        }
                    }

                    if n_level.level == D {
                        self.root = value;
                        return;
                    }

                    n_level.ascend();
                    p_inherit.ascend(&n_level);

                    once = true;
                }

                if once {
                    // Insert the value at the highest level possible such that all values
                    // iterated in the loop above inherit this.
                    values.insert(n_level.key(), value);
                    return;
                }
            }

            // not an inheritor (try to inherit)
            {
                let inheritor = inherit.get_child(base_p_inherit, base_n_level);
                if !inheritor {
                    // check the chain of inheritance of the parent
                    let parent_value = {
                        let mut n_level = base_n_level.ascended();
                        let mut p_inherit = base_p_inherit.ascended(&n_level);
                        Self::scan_inheritance(
                            &mut p_inherit,
                            &mut n_level,
                            inherit,
                            values,
                            &self.root,
                        )
                    };

                    if *parent_value == value {
                        values.remove(&base_n_level.key()).unwrap();
                        inherit.set_child(base_p_inherit, base_n_level, true);

                        // l1 has an implicit 0x00 for the have byte indicating that all l0 nodes are leafs
                        let mut have_val = 0x00;
                        let mut p_inherit = base_p_inherit;
                        let mut p_have = base_p_have;
                        let mut n_level = base_n_level;

                        // try to merge uniform regions
                        loop {
                            let siblings_leafs = have_val == 0x00;
                            let siblings_inherit = inherit[p_inherit] == 0xFF;
                            if !(siblings_leafs && siblings_inherit) {
                                break;
                            }

                            if n_level.level == D - 1 {
                                self.inner = None;
                                return;
                            }

                            n_level.ascend();
                            p_inherit.ascend(&n_level);
                            p_have.ascend(&n_level);

                            have.set_child(p_have, n_level, false);
                            have_val = have[p_have];
                        }
                        return;
                    }
                } else {
                    inherit.set_child(base_p_inherit, base_n_level, false);
                }
                values.insert(base_n_level.key(), value);
            }
        } else {
            let mut p_have = base_p_have;
            let mut p_inherit = base_p_inherit;
            let mut n_level = base_n_level;

            while n_level.level != 0 {
                have.set_child(p_have, n_level, true);

                p_have.descend(&n_level);
                p_inherit.descend(&n_level);
                n_level.descend();
            }

            values.insert(n_level.key(), value);
            inherit.set_child(p_inherit, n_level, false);
        }

        // FIRST if we don't have any inner AND the value is the same as the root, do nothing
        //
        // if we don't have any inner and the value is NOT the same as the root create a new inner with default values
        // then proceed as if we had an inner (because we do)
        //
        // at this point we have an inner:
        // walk our inner.splits towards pos until we find an unsplit node or reach level 0. (pause are these different? the only difference is that we might have to split an unsplit node)
        // if we reach level 0:
        //      walk inner.inherits backwards from our level until we find a non-inherited node or root
        //      compare our value to the read value, if they are the same, do nothing
        //      if they are different:
        //          go to the direct parent of our lvl 0 node, read it's child_inheritance
        //          if child_inheritance == 1 << our_oct {
        //              now we must set the map value at our parent to our value since we were the sole inheritor
        //
        //              ~~*map.insert(parent_key, value) \\ this may override another value OR it may not~~
        //              BUT WAIT... what if our parent is the sole inheritor too?
        //
        //              this must recurse until we see someone who isn't the sole inheritor and set their value
        //
        //              BUT DOULBE WAIT SINCE we just changed a non-leaf nodes value we need to check if any other
        //              values that we're once NOT able to inherit (because they were different) are not equal to their
        //              parent ALSO for performance reasons we should be doing THIS step after every recursion into checking
        //              if our parent is the sole inheritor as well (because at that point we're already sure that the parent in
        //              question will be equal to our value)
        //              You may be like "thats really fucking expensive" and I would be like:
        //              "yes... yes it is... but like super rare... so gotacha"
        //          }
        //          // WE Know we're not the sole inheritor BUT maybe we're not inheriting at all, if so we should check if our operaiton makes it possible to inherit
        //          I'm sorta confused see I wrote this as if we only cared about the direct parent of the node in question but most other
        //          operations recurse into the parents. Well... if we are inheriting from our direct parent then our value is either equal (early ret)
        //          sole inheritor (code above) or the old value is not equal to our direct parents value and we cannot change our direct parents value.
        //          Hence here we see if we need to store our value or if we can elide it.
        //          if child_inheritance >> our_oct & 1 == 0 {
        //              now we must check to see if our new edit can be simplified into an inherit
        //              we don't have to do this check if we are inheriting because we earlier checked the the current value and if we were inheriting it would
        //              be equal one
        //
        //              Since we already know that the value it's set to is NOT equal
        //              if map.get(parent_key) == value { // NOPE we have to recurse through the chain of inheritance not get our parent_key.
        //                  // you may be like "we already recursed through this exact chain" no we didn't the first time we recurse we know
        //                  // we didn't get anywhere past ourself because we just checked that we weren't inheriting. We're basically
        //                  // checking the chain above the chain earlier.
        //                  child_inheritance |= 1 << our_oct
        //                  map.remove(our_key) // we know this exists because an entry already exists (lvl0) and we are not inheriting
        //
        //                  but wait we must now check if the nodes are mergable
        //                  if child_inheritance == u8::MAX {
        //                      well fuck I guess they are: this means we must unsplit the parent. Thankfully we don't have to remove
        //                      any map values because they are guarnateed removed by the fact they were inherited
        //                      split[parent].set(false)
        //                  }
        //              }
        //          }
        //          finally we CANNOT inherit because we know that the inherited value cannot be set equal to our value
        //          else {
        //              // to get here we either know A: we weren't inheriting or B: we were inhering but cannot now inherit
        //              child_inheritance &= !(1 << our_oct);
        //              map.insert(our_key, value);
        //          }
        //
        // Alright say we didn't reach level 0 (shit this might get complicated):
        //      again we walk through the chain of inheritance and check if our current value is equal to the target and if so early return.
        //
        //      at this point we know that the value previously assigned here is NOT equal to the new one.
        //
        //      now here's a problem... we're trying to set a SINGLE cell and right now we're looking at some sorta cube of cells all of which
        //      are equally values as NOT our target value. In this case I think theres no chance of merging and our only option is to
        //      split all the way down the chain and insert a value at lvl0 equal to our target value.
        //
        //      thankfully we don't have to set any of the inheritance values because all inheritance values are true
        //      for merged cubes.
        //
        //      for level in level..0 {
        //          split[idx(pos, lvl)] = true;
        //      }
        //
        //      let bottom_lvl = 0
        //      inheritance[idx(pos, bottom_lvl)] = false;
        //
    }
}

/// Returns the number of nodes in the tree with the given depth.
///
/// Equal to \sum_{d=0}^{D} 8^{d} = \frac{8^{D+1}-1}{7}
const fn tree_len(depth: u32) -> u32 {
    (8u32.pow(depth + 1) - 1) / 7
}

fn get_octant(mut pos: UVec3, level: u32) -> u32 {
    pos >>= level;
    pos &= 1;
    pos.x | pos.y << 1 | pos.z << 2
}

const fn get_child(byte: &u8, octant: u32) -> bool {
    (*byte >> octant) & 1 != 0
}

const fn set_child(byte: &mut u8, octant: u32, value: bool) {
    if value {
        *byte |= 1 << octant;
    } else {
        *byte &= !(1 << octant);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tree_len_matches_geometric_sum() {
        let mut expected = 0;
        let mut nodes_at_level = 1;

        for depth in 0..=8 {
            expected += nodes_at_level;
            assert_eq!(tree_len(depth), expected);
            nodes_at_level *= 8;
        }
    }

    #[test]
    fn cursor_descend_and_ascend_are_inverses() {
        for depth in 1..=8 {
            let root = TreeCursor::root(depth);
            let pos = UVec3::ZERO;
            let mut level = LevelCursor::root(depth, &pos);
            level.descend();

            for octant in 0..8 {
                level.octant = octant;
                let mut cursor = root;
                cursor.descend(&level);
                assert_eq!(cursor.tree_len, tree_len(depth - 1));
                assert_eq!(cursor.index, 1 + octant * tree_len(depth - 1));
                cursor.ascend(&level);
                assert_eq!(cursor, root);
            }
        }
    }

    #[test]
    fn cursor_indexes_all_nodes_once() {
        fn visit(cursor: TreeCursor, depth: u32, indexes: &mut Vec<u32>) {
            indexes.push(cursor.index);
            if depth == 0 {
                return;
            }

            for octant in 0..8 {
                let pos = UVec3::new(
                    (octant & 1) << (depth - 1),
                    ((octant >> 1) & 1) << (depth - 1),
                    ((octant >> 2) & 1) << (depth - 1),
                );
                let child = LevelCursor::root(depth, &pos).descended();
                visit(cursor.descended(&child), depth - 1, indexes);
            }
        }

        for depth in 0..=5 {
            let mut indexes = Vec::new();
            visit(TreeCursor::root(depth), depth, &mut indexes);
            indexes.sort_unstable();
            assert_eq!(indexes, (0..tree_len(depth)).collect::<Vec<_>>());
        }
    }

    #[test]
    fn complete_byte_octree_indexes_are_independent() {
        let mut tree = CompleteByteOctree::new(2, 0);
        let root = TreeCursor::root(2);
        let pos3 = UVec3::new(2, 2, 0);
        let pos5 = UVec3::new(0, 2, 2);
        let child3 = LevelCursor::root(2, &pos3).descended();
        let child5 = LevelCursor::root(2, &pos5).descended();

        tree.set_child(root, child3, true);
        tree.set_child(root, child5, true);

        assert!(tree.get_child(root, child3));
        assert!(tree.get_child(root, child5));
        assert!(!tree.get_child(root, LevelCursor::root(2, &UVec3::ZERO).descended()));
        assert!(!tree.get_child(root, LevelCursor::root(2, &UVec3::new(0, 2, 0)).descended()));
    }

    #[test]
    fn metadata_trees_have_expected_depths() {
        let inner = OctreeInner::<u8>::new(3);

        assert_eq!(inner.children_have_children.vec.len(), tree_len(1) as usize);
        assert_eq!(inner.children_inherit_this.vec.len(), tree_len(2) as usize);
    }

    #[test]
    fn octree_get_returns_root_and_set_values() {
        let mut tree = Octree::<u8, 3>::new(0);
        let first = UVec3::new(0, 0, 0);
        let second = UVec3::new(7, 3, 5);

        assert_eq!(*tree.get(first), 0);
        assert_eq!(*tree.get(second), 0);

        tree.set(first, 1);
        tree.set(second, 2);

        assert_eq!(*tree.get(first), 1);
        assert_eq!(*tree.get(second), 2);
    }

    #[test]
    fn octree_preserves_values_for_all_octants() {
        let mut tree = Octree::<u8, 3>::new(0);

        for octant in 0..8 {
            let pos = UVec3::new(
                (octant & 1) * 4,
                ((octant >> 1) & 1) * 4,
                ((octant >> 2) & 1) * 4,
            );
            tree.set(pos, octant as u8 + 1);
        }

        for octant in 0..8 {
            let pos = UVec3::new(
                (octant & 1) * 4,
                ((octant >> 1) & 1) * 4,
                ((octant >> 2) & 1) * 4,
            );
            assert_eq!(*tree.get(pos), octant as u8 + 1);
        }
    }

    #[test]
    fn octree_round_trips_deterministic_random_volume() {
        let mut tree = Octree::<u8, 3>::new(0);
        let mut expected = [0u8; 64];
        let mut state = 0x9e37_79b9u32;

        for value in &mut expected {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *value = (state % 11) as u8;
        }

        for (index, &value) in expected.iter().enumerate() {
            let pos = UVec3::new(
                (index % 4) as u32,
                ((index / 4) % 4) as u32,
                (index / 16) as u32,
            );
            tree.set(pos, value);
        }

        let mut round_trip = [0u8; 64];
        for (index, value) in round_trip.iter_mut().enumerate() {
            let pos = UVec3::new(
                (index % 4) as u32,
                ((index / 4) % 4) as u32,
                (index / 16) as u32,
            );
            *value = *tree.get(pos);
        }

        assert_eq!(round_trip, expected);
    }

    #[test]
    fn octree_set_exercises_merge_transitions() {
        let mut tree = Octree::<u8, 3>::new(0);
        let a = UVec3::new(0, 0, 0);
        let b = UVec3::new(0, 0, 1);
        let c = UVec3::new(0, 1, 0);
        let d = UVec3::new(0, 1, 1);

        tree.set(a, 1);
        assert_eq!(*tree.get(a), 1);

        tree.set(b, 2);
        assert_eq!(*tree.get(a), 1);
        assert_eq!(*tree.get(b), 2);

        tree.set(b, 1);
        assert_eq!(*tree.get(a), 1);
        assert_eq!(*tree.get(b), 1);

        tree.set(c, 3);
        tree.set(d, 3);
        assert_eq!(*tree.get(c), 3);
        assert_eq!(*tree.get(d), 3);

        tree.set(c, 1);
        assert_eq!(*tree.get(c), 1);
        assert_eq!(*tree.get(d), 3);

        tree.set(a, 0);
        tree.set(b, 0);
        assert_eq!(*tree.get(a), 0);
        assert_eq!(*tree.get(b), 0);
        assert_eq!(*tree.get(c), 1);
        assert_eq!(*tree.get(d), 3);
    }
}
