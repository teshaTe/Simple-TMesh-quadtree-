#ifndef H_TMESH_CLASS
#define H_TMESH_CLASS

#include <glm/glm.hpp>

#include <array>
#include <vector>
#include <memory>

namespace tmesh2D {

enum class Direction
{
    NORTH,
    EAST,
    WEST,
    SOUTH,
    NORTH_WEST,
    NORTH_EAST,
    SOUTH_EAST,
    SOUTH_WEST
};

class TMeshBoundingBox
{
public:
    TMeshBoundingBox(glm::f32 Left = 0, glm::f32 Top = 0, glm::f32 Width = 0, glm::f32 Height=0) noexcept:
                     left(Left), top(Top), width(Width), height(Height) { }
    TMeshBoundingBox(const glm::f32vec2 pos, const glm::f32vec2 size) noexcept:
                     left(pos.x), top(pos.y), width(size.x), height(size.y) { }

    constexpr glm::f32 getRight()          const noexcept { return left + width; }
    constexpr glm::f32 getBottom()         const noexcept { return top + height; }
    constexpr glm::f32vec2 getTopLeft()    const noexcept { return glm::f32vec2(left, top); }
    constexpr glm::f32vec2 getCenter()     const noexcept { return glm::f32vec2(left+width/2.0f, top+height/2.0f); }
    constexpr glm::f32vec2 getSize()       const noexcept { return glm::f32vec2(width, height); }

    constexpr bool contains(const TMeshBoundingBox &box) const noexcept
    {
        return left <= box.left && box.getRight()  <= getRight() &&
                top <= box.top  && box.getBottom() <= getBottom();
    }

    glm::f32 vals[4];
    friend class TMesh;

private:
    glm::f32 left, top;
    glm::f32 width, height;

}; // class TMeshBoundingBox


class TMesh : public TMeshBoundingBox
{
public:
    struct tNode
    {
        // children nodes stored as pointers;
        std::array<std::unique_ptr<tNode>, 4> children{nullptr};

        // neighbours of child nodes; stored as pointers;
        // 0 - north, 1 - west; 2 - east, 3 - south neighbours;
        std::array<std::vector<tNode*>, 4> neighbours;

        // parent node of the current tNode;
        tNode *parent;

        // values are the grid points that are in the current node;
        // values.x = value; values.yz - coordinates of the point containing the value;
        std::vector<glm::f32vec3> values;

        // node cell geometry;
        TMeshBoundingBox nodeBox;
        int dLevel{-1};
    };

public:
    /**
     * @brief TMesh  - class constructor for the  quadtree that is used for scalar fields
     * @param box       - predefined bounding box
     * @param resX      - width of the passing grid to be segmented by quadtree
     * @param resY      - heght of the passing grid to be segmented by quadtree
     * @param threshold - value that controlls the amount of stored frep values in one node
     * @param maxDepth  - maximum depth of the building tree
     * @param maxD_ext  - maximum depth for the exterior of the object
     * @param maxD_int  - maximum depth fot the interior of the object
     */
    TMesh(const TMeshBoundingBox &box, const int resX, const int resY, const size_t threshold,
          const size_t maxD_surf, const size_t maxD_ext, const size_t maxD_int):
          m_Root(std::make_unique<tNode>()), m_Box(box), m_Depth(maxD_surf), m_thres(threshold), m_resX(resX), m_resY(resY)
    {
        m_depthExterior = maxD_ext;
        m_depthInterior = maxD_int;
        m_Root->parent = nullptr;
    }

    TMesh(TMesh && otherTmesh) = default;
    TMesh &operator=(TMesh && otherTMesh) = default;

    TMesh()  = default;
    ~TMesh() = default;

    /**
     * @brief generateTMesh - function that starts the generation process of TMesh construction
     *        using quadtree space subdivision. It is also constructing PHT-spline basis functions
     *        at each cell, thtat can be further used for field restoration at each cell.
     * @param inField - FRep or any other function-based field that defines the object
     * @param surfLim - ctiteria for identifying zero-level set (narrow band),
     *        .x - is usually 0 or -10^-3 or similar; .y - 10^-3 or similar
     * @param phtSp - PHT-spline class that is used for computing PHT-spline basis functions at each cell
     */
    void generateTMesh(const std::vector<float> *inField, const glm::vec2 surfLim);

    /**
     * @brief checkTMesh - function that make simple check of the obtained tree data-structure
     */
    void checkTMesh();

    /**
     * @brief getNeighbours - searching for all neighbour nodes of the passed node.
     * @param node      - node for which we are looking for the neighbours in the tree structure;
     * @param direction - helps to control in which direction we are searching for the neighbour;
     * @return a vector of found neighbours in particular direction;
     */
    std::vector<tNode *> getNeighbours(tNode *node, Direction direction);

    /**
     * @brief getLeafs - find all leafs of the constructed TMesh
     * @return a vector of found leafs
     */
    std::vector<tNode *> getLeaves();

    inline bool isLeaf(tNode *node) const { return !static_cast<bool>(node->children[0]); }

    inline void insertNode(const glm::f32vec2 pos, const glm::f32 &value)
                           { insert(m_Root.get(), 0, m_Box, pos, value); }

    inline void removeNode(const glm::f32vec2 pos, const glm::f32 &value)
                           { remove(m_Root.get(), nullptr, m_Box, pos, value); }

    inline std::vector<tNode*> getLeavesWithFunVals(const std::vector<float> *inField)
                                                   { return getLeavesWithVal(inField); }

    inline tNode* getRoot()      { return m_Root.get(); }
    inline int getMaxDepth()     { return m_Depth; }

private:
    /**
     * @brief computeBox - function for computing new bounding boxes on the fly
     *                     it computes the box of a child from the 'box' of the parent and index 'i' of the quadrant
     * @param box - bounding box that will be further subdivided;
     * @param i   - the quadrant index: 0 - north west, 1 - north east, 2 - south west, 3 - south east;
     * @return - returns the subdivided bounding box;
     */
    TMeshBoundingBox computeBox(const TMeshBoundingBox &box, int i) const;

    /**
     * @brief getBoundingBox - creating bounding box with the smallest size using current position;
     * @param pos - position of the point around which we create a tiny bounding box;
     * @return created tiny bounding box around point defined using pos.
     */
    TMeshBoundingBox getBoundingBox(glm::f32vec2 pos) { return TMeshBoundingBox(pos, glm::f32vec2(0, 0)); }


    /**
     * @brief getQuadrant - checking if the passed bounding box is fully contained in any quadrant;
     * @param nodeBox - is the passed parent box inside of which we are trying to find valBox;
     * @param valBox  - is an input bounding box that we are trying to find in one of the quadrants;
     * @return - the index of the quadrant or -1 if passed bounding box is not entirely contained in any quadrant
     */
    int getQuadrant(const TMeshBoundingBox &nodeBox, const TMeshBoundingBox &valBox) const;

    /**
     * @brief insert - function that insert new node to the tree;
     * @param node  - node to be inserted in the tree;
     * @param depth - the depth of the tree at which the node will be inserted;
     * @param box   - bounding box;
     * @param pos   - position of the point in which value is defined;
     * @param val   - value defined in the point with coords <pos>.
     * TODO: add pht spline
     */
    void insert(tNode* node, size_t depth, const TMeshBoundingBox &box, const glm::f32vec2 pos, const glm::f32 &val);

    /**
     * @brief split - splitting the parent node into 4 child nodes and
     *                computing new PHT-basis functions;
     * @param node - node to split;
     * @param box  - bouding box to split.
     */
    void split(tNode *node, const TMeshBoundingBox &box);

    /**
     * @brief remove - function for removing node and value
     * @param node
     * @param parent
     * @param box
     * @param pos
     * @param val
     */
    void remove(tNode *node, tNode *parent, const TMeshBoundingBox &box, const glm::f32vec2 pos, const glm::f32 &val);
    /**
     * @brief removeValue
     * @param node
     * @param pos
     * @param val
     */
    void removeValue(tNode *node, const glm::f32vec2 pos, const glm::f32 &val);

    /**
     * @brief tryMergeNodes - merging 4 children in one parent;
     * @param node - passed node with 4 children to be merged in a parent node.
     */
    void tryMergeNodes(tNode *node);

    /**
     * @brief getLeafsWithVal - function for obtaining leafs with stored values of the input
     *                          function-based field at each corner of the cell after subdivision of space;
     * @param inField - defining function field of the object that was subdivided using quadtree -> TMesh;
     * @return a vector of leafs with stores inField values at each corner of each TMesh cell.
     */
    std::vector<tNode *> getLeavesWithVal(const std::vector<float> *inField);

    /**
     * @brief findLeafs - searching for all leafs of the computed TMesh;
     * @param node - root node of the tree;
     * @param box - root bounding box of the tree;
     * @return all leafs in one vector.
     */
    std::vector<tNode *> findLeaves(tNode *node , TMeshBoundingBox box);

    //void updateLeafVals(tNode *node, tNode*leaf);
    //void copy(tNode *inNode, tNode* outNode);

    //********************************************************************************
    // direction - direction where the current cell is: 0 [NW], 1 [NE], 2 [SW], 3 [SE]
    //********************************************************************************
    /**
     * @brief findSmallerNeighbours - searching for the neighbours with the bounding box size
     *                                smaller comparing to the passed node bounding box size;
     * @param node      - the candidate node for which we want to find the neighbours;
     * @param direction - one of the possible directions to look for the neighbours;
     * @return a vector of neighbours with bounding box of the smaller size.
     */
    std::vector<tNode *> findSmallerNeighbours(tNode *node, Direction direction);

    /**
     * @brief findGreaterEqualNeighbour - searching for a node with the bounding box size
     *                                    greater or similar to the passed node bounding box size;
     * @param node      - the candidate node for which we want to find the neighbours;
     * @param direction - one of the possible directions to look for the neighbours;
     * @return one found neighbour with bounding box of the same or greater size.
     */
    tNode *findGreaterEqualNeighbour(tNode *node, Direction direction);

private:
    std::unique_ptr<tNode> m_Root;
    std::vector<tNode*> m_leaves;
    tNode *m_parent;

    size_t m_thres, m_Depth;
    size_t m_depthExterior, m_depthInterior;
    glm::f32vec2 m_surfLim;

    int m_resX, m_resY;
    int m_l{0};

    TMeshBoundingBox m_Box;

}; // class TMesh

} // namespace tmesh2D

#endif // define H_TMESH_CLASS
