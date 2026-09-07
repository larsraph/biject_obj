use bevy::{
    math::{U8Vec3, UVec3},
    platform::collections::HashMap,
};

/// Manages persistent state when walking through a octree depth-first with
/// depth `D` and no leaf data.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct Cursor<const D: u32> {
    idx: u32,
    level: u32,
}

impl<const D: u32> Cursor<D> {
    const fn root() -> Self {
        Cursor { idx: 0, level: D }
    }

    fn octant(&self, pos: UVec3) -> u32 {
        octant(pos, self.level)
    }

    fn next(mut self, oct: u32) -> Self {
        self.level -= 1;
        self.idx += 1 + oct * tree_len_no_leaves(self.level);
        self
    }

    fn toward(self, pos: UVec3) -> Self {
        self.next(octant(pos, self.level))
    }

    fn toward_level(mut self, pos: UVec3, level: u32) -> Self {
        while self.level > level {
            self = self.toward(pos);
        }
        self
    }

    fn prev(mut self, oct: u32) -> Self {
        self.level += 1;
        self.idx -= 1 + oct * tree_len_no_leaves(self.level);
        self
    }

    fn away(self, pos: UVec3) -> Self {
        self.prev(octant(pos, self.level))
    }

    fn is_root(&self) -> bool {
        self.level == D
    }

    fn is_in_bounds(&self) -> bool {
        self.level <= D
    }

    fn key(&self, pos: UVec3) -> Key {
        Key::new(pos, self.level)
    }
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

    #[inline]
    fn get_byte(&self, cursor: Cursor<D>) -> u8 {
        self.vec[cursor.idx as usize]
    }

    #[inline]
    fn byte_mut(&mut self, cursor: Cursor<D>) -> &mut u8 {
        &mut self.vec[cursor.idx as usize]
    }

    #[inline]
    fn get(&self, cursor: Cursor<D>, oct: u32) -> bool {
        (self.get_byte(cursor) >> oct) & 1 != 0
    }

    #[inline]
    fn set(&mut self, cursor: Cursor<D>, value: bool, oct: u32) {
        if value {
            self.vec[cursor.idx as usize] |= 1 << oct;
        } else {
            self.vec[cursor.idx as usize] &= !(1 << oct);
        }
    }

    #[inline]
    fn scan_away(&self, mut cursor: Cursor<D>, pos: UVec3) -> Cursor<D> {
        while cursor.is_in_bounds() {
            if !self.get(cursor, cursor.octant(pos)) {
                return cursor;
            }

            cursor = cursor.away(pos);
        }
        Cursor::root()
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
        let len_no_root = len - 1;
        debug_assert_eq!(len_no_root % 8, 0);
        let byte_len = (len_no_root / 8) as usize;
        let vec = vec![0; byte_len];
        Self { vec }
    }

    fn get(&self, cursor: Cursor<D>) -> bool {
        debug_assert!(!cursor.is_root());
        let idx = cursor.idx - 1;
        let byte_idx = idx / 8;
        let bit_idx = idx % 8;
        let byte = self.vec[byte_idx as usize];
        (byte >> bit_idx) & 1 != 0
    }

    fn set(&mut self, cursor: Cursor<D>, value: bool) {
        debug_assert!(!cursor.is_root());
        let idx = cursor.idx - 1;
        let byte_idx = idx / 8;
        let bit_idx = idx % 8;
        let byte = &mut self.vec[byte_idx as usize];
        if value {
            *byte |= 1 << bit_idx;
        } else {
            *byte &= !(1 << bit_idx);
        }
    }

    fn scan_towards_from_root(&self, pos: UVec3) -> Cursor<D> {
        let mut cursor = Cursor::root();
        cursor.toward(pos);
        while cursor.level > 1 {
            if !self.get(cursor) {
                return cursor;
            }
            cursor = cursor.toward(pos);
        }
        cursor
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

    fn with_oct(mut self, oct: u32) -> Self {
        self.pos &= !1;
        self.pos.x |= (oct & 1) as u8;
        self.pos.y |= ((oct >> 1) & 1) as u8;
        self.pos.z |= ((oct >> 2) & 1) as u8;
        self
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

        let cursor = inner.this_has_children.scan_towards_from_root(pos);
        let inheritee = inner.children_inherit_this.scan_away(cursor, pos);

        if inheritee.is_root() {
            self.root
        } else {
            *inner.values.get(&inheritee.key(pos)).unwrap()
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

        let cursor = inner.this_has_children.scan_towards_from_root(pos);
        let c_oct = cursor.octant(pos);
        let inheritee = inner.children_inherit_this.scan_away(cursor, pos);

        let old_value = if inheritee.is_root() {
            self.root
        } else {
            *inner.values.get(&inheritee.key(pos)).unwrap()
        };
        if old_value == value {
            return;
        }

        if cursor.level == 0 {
            let parent = cursor.away(pos);
            let parent_byte = inner.children_inherit_this.get_byte(parent);

            {
                let mut ancestor = parent;
                let mut cursor = cursor;
                let mut oct = c_oct;
                let mut ancestor_byte = parent_byte;
                let mut commit = false;

                loop {
                    if ancestor_byte != 1 << oct {
                        break;
                    }
                    commit = true;
                    cursor = ancestor;
                    ancestor = cursor.away(pos);
                    oct = cursor.octant(pos);
                    ancestor_byte = inner.children_inherit_this.get_byte(ancestor);

                    // reevaluate children
                    for oct in 0..8 {
                        let cursor = ancestor.next(oct);
                        let key = cursor.key(pos).with_oct(oct);

                        let other_value = *inner.values.get(&key).unwrap();
                        if value == other_value {
                            inner.values.remove(&key);
                            inner.children_inherit_this.set(cursor, true, oct);
                        }
                    }
                }

                if commit {
                    if cursor.is_root() {
                        self.root = value;
                    } else {
                        inner.values.insert(cursor.key(pos), value);
                    }
                    return;
                }
            }

            let doesnt_inherit = (parent_byte >> c_oct) & 1 == 0;
            if doesnt_inherit {
                let parent_inheritee = inner.children_inherit_this.scan_away(parent, pos);
                let parent_value = *inner.values.get(&parent_inheritee.key(pos)).unwrap();
                if parent_value == value {
                    inner.values.remove(&cursor.key(pos)).unwrap();

                    *inner.children_inherit_this.byte_mut(parent) |= 1 << c_oct;

                    let mut ancestor = parent;
                    let mut anc_byte = parent_byte;

                    loop {
                        if anc_byte != !0u8 {
                            break;
                        }

                        let mut any_have = false;
                        for oct in 0..8 {
                            let anc_child = ancestor.next(oct);
                            any_have |= inner.this_has_children.get(anc_child);
                        }
                        if any_have {
                            break;
                        }

                        if ancestor.is_root() {
                            self.inner = None;
                            return;
                        } else {
                            inner.this_has_children.set(ancestor, false);
                        }

                        ancestor = ancestor.away(pos);
                        anc_byte = inner.children_inherit_this.get_byte(ancestor);
                    }

                    return;
                }
            }

            inner.children_inherit_this.set(parent, false, c_oct);
            inner.values.insert(cursor.key(pos), value);
        } else {
            let mut last = cursor;
            let mut drill = cursor;
            while drill.level != 0 {
                inner.this_has_children.set(drill, true);
                last = drill;
                drill = drill.toward(pos);
            }
            inner.values.insert(cursor.key(pos), value);
            let oct = drill.octant(pos);
            inner.children_inherit_this.set(last, false, oct)
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
