use bevy::{
    math::{U8Vec3, UVec3},
    platform::collections::HashMap,
};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct Cursor {
    idx: u32,
    level: u32,
}

#[derive(Clone, Debug)]
struct ChildrenInheritThis<const D: u32> {
    vec: Vec<u8>,
}

impl<const D: u32> ChildrenInheritThis<D> {
    #[inline]
    fn new() -> Self {
        let len = tree_len_no_leaves(D);
        let vec = vec![0; len as usize];
        Self { vec }
    }

    fn cursor() -> Cursor {
        Cursor { idx: 0, level: D }
    }

    #[inline]
    fn walk_back_from(&self, pos: UVec3, level: u32) -> u32 {
        let mut parent_idx = 0;
        for level in (level + 1..=D - 1).rev() {
            let oct = octant(pos, level);
            let tree_len = tree_len_no_leaves(level);
            parent_idx += 1 + oct * tree_len;
        }

        for level in level..=D - 1 {
            let oct = octant(pos, level);
            if !self.this_has_child(parent_idx, oct) {
                return level;
            }

            if level == D - 1 {
                break;
            }

            let parent_level = level + 1;
            let parent_oct = octant(pos, parent_level);
            let parent_tree_len = tree_len_no_leaves(parent_level);
            parent_idx -= 1 + parent_oct * parent_tree_len;
        }
        // if no levels don't inherit return the root level
        D
    }

    fn this_has_child(&self, idx: u32, oct: u32) -> bool {
        let byte = self.vec[idx as usize];
        (byte >> oct) & 1 != 0
    }
}

#[derive(Clone, Debug)]
struct ThisHasChildren<const D: u32> {
    vec: Vec<u8>,
}

impl<const D: u32> ThisHasChildren<D> {
    #[inline]
    fn new() -> Self {
        let len = tree_len_no_leaves(D);
        // the root is elided
        let len_no_root = len - 1;
        // a octree with no root must have the number of nodes divisible by 8
        debug_assert_eq!(len_no_root % 8, 0);
        let byte_len = (len_no_root / 8) as usize;
        let vec = vec![0; byte_len];
        Self { vec }
    }

    #[inline]
    fn walk_towards(&self, pos: UVec3) -> u32 {
        let mut idx = 0;
        // The order is [D - 1, D - 2, ..., 1]. Level D is the root.
        for level in (1..=(D - 1)).rev() {
            let oct = octant(pos, level);
            let tree_len = tree_len_no_leaves(level);
            idx += oct * tree_len;

            if !self.has_children(idx) {
                return level;
            }

            // by adding 1 AFTER we query we don't get any funny business
            // from the fact that idx 0 (aka the root) is omitted
            idx += 1;
        }
        // if no levels didn't have children return level 0 the leaf level
        0
    }

    #[inline]
    fn has_children(&self, idx: u32) -> bool {
        let byte_idx = idx / 8;
        let bit_idx = idx % 8;
        let byte = self.vec[byte_idx as usize];
        (byte >> bit_idx) & 1 != 0
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct Key {
    pos: U8Vec3,
    level: u8,
}

impl Key {
    #[inline]
    fn new(pos: UVec3, level: u32) -> Self {
        let level = level as u8;
        let pos = (pos >> level).as_u8vec3();
        Self { level, pos }
    }
}

#[derive(Clone, Debug)]
struct OctreeInner<T, const D: u32> {
    values: HashMap<Key, T>,
    this_has_children: ThisHasChildren<D>,
    children_inherit_this: ChildrenInheritThis<D>,
}

impl<T, const D: u32> OctreeInner<T, D> {
    #[inline]
    fn new() -> Self {
        Self {
            values: HashMap::new(),
            this_has_children: ThisHasChildren::new(),
            children_inherit_this: ChildrenInheritThis::new(),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Octree<T: Copy + PartialEq, const D: u32 = 5> {
    root: T,
    inner: Option<OctreeInner<T, D>>,
}

impl<T: Copy + PartialEq, const D: u32> Octree<T, D> {
    pub fn new(root: T) -> Self {
        Self { root, inner: None }
    }

    pub fn get(&self, pos: UVec3) -> T {
        let Some(inner) = &self.inner else {
            return self.root;
        };

        let level = inner.this_has_children.walk_towards(pos);
        let level = inner.children_inherit_this.walk_back_from(pos, level);

        if level == D {
            self.root
        } else {
            *inner.values.get(&Key::new(pos, level)).unwrap()
        }

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
        let inner = match &mut self.inner {
            Some(inner) => inner,
            None => {
                if value == self.root {
                    return;
                }
                self.inner.insert(OctreeInner::new())
            }
        };

        let level = inner.this_has_children.walk_towards(pos);
        if level == 0 {
            let level = inner.children_inherit_this.walk_back_from(pos, level);

            let old_value = if level == D {
                self.root
            } else {
                *inner.values.get(&Key::new(pos, level)).unwrap()
            };
            if old_value == value {
                return;
            }

            let oct = octant(pos, level);
            let parent_level;
            if inner.children_inherit_this.get(parent_idx) == 1 << oct {
                //
            }
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
#[inline]
const fn tree_len(depth: u32) -> u32 {
    (8u32.pow(depth + 1) - 1) / 7
}

/// Equivalent to [`subtree_len`]
const fn tree_len_no_leaves(depth: u32) -> u32 {
    subtree_len(depth)
}

/// Returns the number of nodes in the subtree with the given depth.
///
/// Equal to \sum_{d=0}^{D - 1} 8^{d} = \frac{8^D-1}{7}
const fn subtree_len(depth: u32) -> u32 {
    (8u32.pow(depth) - 1) / 7
}

#[inline]
fn octant(mut pos: UVec3, level: u32) -> u32 {
    pos >>= level;
    pos &= 1;
    pos.x | pos.y << 1 | pos.z << 2
}
