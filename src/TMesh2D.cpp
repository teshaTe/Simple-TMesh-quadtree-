#include "include/timer.hpp"
#include "include/TMesh2D.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <list>

namespace tmesh2D {

void TMesh::generateTMesh(const std::vector<float> *inField, const glm::vec2 surfLim)
{
    prof::timer time;
    time.Start();

    m_surfLim = surfLim;
    for(int y = 0; y < m_resY; y++)
        for(int x = 0; x < m_resX; x++)
            insertNode(glm::f32vec2(x, y), inField->at(x+y*m_resX));

    time.End("TMesh building time");
    getLeavesWithFunVals(inField);
}

void TMesh::checkTMesh()
{
    std::cout << "Start checking T-Mesh:" << std::endl;
    m_leaves = findLeaves(m_Root.get(), m_Box);

    if(m_leaves.empty())
        throw std::string("ERROR: T-Mesh does not have leafs!");

    // looking for leafs with empty "values" array;
    int empty = 0;
    for(auto leaf : m_leaves)
    {
        if(leaf->values.empty())
            empty++;
    }

    if(empty > 0)
        throw std::string("ERROR: Some leafs have empty 'values' array!");

    std::cout << "Finished checking T-Mesh: no erros were found." << std::endl;
}

void TMesh::insert(TMesh::tNode *node, size_t depth, const TMeshBoundingBox &box,
                   const glm::f32vec2 pos, const glm::f32 &val)
{
    assert(node != nullptr);
    assert(box.contains(getBoundingBox(pos)) && "ASSERT: Value is not in this box!");

    if(isLeaf(node))
    {
        // insert value in this node if possible
        if(node->values.size() < m_thres)
        {
            node->values.push_back(glm::f32vec3(val, pos.x, pos.y));
            node->dLevel = depth;
        }
        else {
            if(val >= m_surfLim.x && val <= m_surfLim.y && depth < m_Depth)
            {
                split(node, box);
                insert(node, depth, box, pos, val);
            }
            else if(val <= m_surfLim.x && depth < m_depthExterior)
            {
                split(node, box);
                insert(node, depth, box, pos, val);
            }
            else if(val >= m_surfLim.x && depth < m_depthInterior)
            {
                split(node, box);
                insert(node, depth, box, pos, val);
            }
            else {
                node->values.push_back(glm::f32vec3(val, pos.x, pos.y));
                node->dLevel = depth;
            }
        }
    }
    //otherwise we have a branch and we need to add new node pointers to children
    else {
        auto i = getQuadrant(box, getBoundingBox(pos));
        // Add the value in a child if the value is entirely contained in it
        if(i != -1)
            insert(node->children[static_cast<size_t>(i)].get(), depth+1, computeBox(box, i), pos, val);
        // Otherwise, we add the value in the current node
        else {
            node->values.push_back(glm::f32vec3(val, pos.x, pos.y));
            node->dLevel = depth;
        }
    }
}

void TMesh::split(TMesh::tNode *node, const TMeshBoundingBox &box)
{
    //only leafs can be splitted
    assert(node != nullptr);
    assert(isLeaf(node) && "ASSERT: Splitting leaves only!");

    //create children
    for(auto& child : node->children)
    {
        child = std::make_unique<tNode>();
        child.get()->parent = node;
    }

    //filling in new values
    auto newValues = std::vector<glm::f32vec3>();

    for(const auto &value : node->values)
    {
        glm::f32vec2 pos(value.y, value.z);
        auto i = getQuadrant(box, getBoundingBox(pos));
        //if the quadrant index found, push down the tree the values of the current node;
        if(i != -1)
        {
            node->children[static_cast<size_t>(i)].get()->values.push_back(value);
            node->children[static_cast<size_t>(i)].get()->dLevel = node->dLevel+1;
        }
        else
            newValues.push_back(value);
    }
    node->values = std::move(newValues);
}

void TMesh::remove(TMesh::tNode *node, TMesh::tNode *parent, const TMeshBoundingBox &box,
                   const glm::f32vec2 pos, const glm::f32 &val)
{
    assert(node != nullptr);
    assert(box.contains(getBoundingBox(pos)));

    if(isLeaf(node))
    {
        //removing the value from the node
        removeValue(node, pos, val);
        // try to merge the parent
        if(parent != nullptr)
            tryMergeNodes(parent);
    }
    else {
        //remove the value in a child if the value entirely contained in it
        int i = getQuadrant(box, getBoundingBox(pos));
        if(i != -1)
            remove(node->children[static_cast<size_t>(i)].get(), node, computeBox(box, i), pos, val);
        else
            //otherwise removing the value from the current node
            removeValue(node, pos, val);
    }
}

void TMesh::removeValue(TMesh::tNode *node, const glm::f32vec2 pos, const glm::f32 &val)
{
    //find the value in node->values
    glm::f32vec3 nodeVal(val, pos);
    auto it = std::find_if(std::begin(node->values), std::end(node->values),
                           [this, &nodeVal](const auto& rhs) { return nodeVal == rhs; });

    assert(it != std::end(node->values) && "ASSERT: trying to remove value that is not presented in the node!");

    //swap with the last element and pop back as the order of values is not important
    *it = std::move(node->values.back());
    node->values.pop_back();
}

void TMesh::tryMergeNodes(TMesh::tNode *node)
{
    assert(node != nullptr);
    assert(!isLeaf(node) && "ASSERT: only interior nodes can be merged!");
    size_t nbValuesSize = node->values.size();

    for(const auto &child : node->children)
    {
        if(!isLeaf(child.get()))
            return;
        else
            nbValuesSize += child->values.size();
    }

    if(nbValuesSize <=- m_thres)
    {
        node->values.reserve(nbValuesSize);
        //merge the values of all children
        for(const auto &child : node->children)
        {
            for(const auto &value : child.get()->values)
                node->values.push_back(value);
        }
        //remove the children
        for(auto &child : node->children)
            child.reset();
    }
}

std::vector<TMesh::tNode *> TMesh::getLeavesWithVal(const std::vector<float> *inField)
{
    m_leaves.clear();
    m_leaves = findLeaves(m_Root.get(), m_Box);

    for(auto leaf : m_leaves)
    {
        size_t xtL = leaf->nodeBox.left,
               ytL = leaf->nodeBox.top;
        size_t xtR = leaf->nodeBox.getRight() - 1,
               ytR = leaf->nodeBox.top;
        size_t xbL = leaf->nodeBox.left - 1,
               ybL = leaf->nodeBox.getBottom() - 1;
        size_t xbR = leaf->nodeBox.getRight() - 1,
               ybR = leaf->nodeBox.getBottom() - 1;

        if(xbR + ybR * m_resX >= m_resX * m_resY)
            throw("ERROR: Out of range");

        leaf->nodeBox.vals[0] = inField->at(xtL + ytL * m_resX);
        leaf->nodeBox.vals[1] = inField->at(xtR + ytR * m_resX);
        leaf->nodeBox.vals[2] = inField->at(xbL + ybL * m_resX);
        leaf->nodeBox.vals[3] = inField->at(xbR + ybR * m_resX);
    }

    return m_leaves;
}

std::vector<TMesh::tNode *> TMesh::findLeaves(TMesh::tNode *node, TMeshBoundingBox box)
{
    TMeshBoundingBox newBox;

    if(isLeaf(node))
    {
        std::vector<TMesh::tNode*> n_north = getNeighbours(node, Direction::NORTH);
        std::vector<TMesh::tNode*> n_east  = getNeighbours(node, Direction::EAST);
        std::vector<TMesh::tNode*> n_west  = getNeighbours(node, Direction::WEST);
        std::vector<TMesh::tNode*> n_south = getNeighbours(node, Direction::SOUTH);

        std::sort(n_north.begin(), n_north.end(), [](auto && l, auto && r)
                      { return l->nodeBox.getTopLeft().x < r->nodeBox.getTopLeft().x; });
        std::sort(n_south.begin(), n_south.end(), [](auto && l, auto && r)
                      { return l->nodeBox.getTopLeft().x < r->nodeBox.getTopLeft().x; });
        std::sort(n_east.begin(), n_east.end(), [](auto && l, auto && r)
                      { return l->nodeBox.getTopLeft().y < r->nodeBox.getTopLeft().y; });
        std::sort(n_west.begin(), n_west.end(), [](auto && l, auto && r)
                      { return l->nodeBox.getTopLeft().y < r->nodeBox.getTopLeft().y; });

        node->neighbours[0] = n_north;
        node->neighbours[1] = n_east;
        node->neighbours[2] = n_west;
        node->neighbours[3] = n_south;

        node->nodeBox = box;
        m_leaves.push_back(node);
    }
    else
    {
        newBox = computeBox(box, 0);
        findLeaves(node->children[0].get(), newBox);

        newBox = computeBox(box, 1);
        findLeaves(node->children[1].get(), newBox);

        newBox = computeBox(box, 2);
        findLeaves(node->children[2].get(), newBox);

        newBox = computeBox(box, 3);
        findLeaves(node->children[3].get(), newBox);
    }

    return m_leaves;
}

std::vector<TMesh::tNode *> TMesh::findSmallerNeighbours(TMesh::tNode *neighbour, Direction direction)
{
    std::vector<TMesh::tNode *> neighbours;
    std::list<TMesh::tNode *> candidates;

    //simple selection of the nodes that can be possible candidates for the neighbours
    if(neighbour == nullptr)
         candidates.clear();
    else
         candidates.push_back(neighbour);

    // Searching for the neighbours in one of the specified directions
    switch(direction)
    {
        case Direction::NORTH:
        {
            while(candidates.size() > 0)
            {
                if(isLeaf(candidates.front()))
                    neighbours.push_back(candidates.front());
                else {
                    candidates.push_back(candidates.front()->children[2].get());
                    candidates.push_back(candidates.front()->children[3].get());
                }
                candidates.pop_front();
            }
            break;
        }

        case Direction::SOUTH:
        {
            while(candidates.size() > 0)
            {
                if(isLeaf(candidates.front()))
                    neighbours.push_back(candidates.front());
                else {
                    candidates.push_back(candidates.front()->children[0].get());
                    candidates.push_back(candidates.front()->children[1].get());
                }
                candidates.pop_front();
            }
            break;
        }

        case Direction::WEST:
        {
            while(candidates.size() > 0)
            {
                if(isLeaf(candidates.front()))
                    neighbours.push_back(candidates.front());
                else {
                    candidates.push_back(candidates.front()->children[1].get());
                    candidates.push_back(candidates.front()->children[3].get());
                }
                candidates.pop_front();
            }
            break;
        }

        case Direction::EAST:
        {
            while(candidates.size() > 0)
            {
                if(isLeaf(candidates.front()))
                    neighbours.push_back(candidates.front());
                else {
                    candidates.push_back(candidates.front()->children[0].get());
                    candidates.push_back(candidates.front()->children[2].get());
                }
                candidates.pop_front();
            }
            break;
        }
        default:
        {
            neighbours.clear();
            throw("ERROR: Unknown direction option!");
        }
    }

    return neighbours;
}

TMesh::tNode *TMesh::findGreaterEqualNeighbour(TMesh::tNode *node, Direction direction)
{
    if(node->parent == nullptr)
        return nullptr;

    switch(direction)
    {
        //find northern neighbour
        case Direction::NORTH:
        {
            if(node->parent->children[2].get() == node) //is node SW child
                return node->parent->children[0].get();

            if(node->parent->children[3].get() == node) //is node SE child
                return node->parent->children[1].get();

            tNode *nNode = findGreaterEqualNeighbour(node->parent, direction);
            if(nNode == nullptr || isLeaf(nNode))
                return nNode;

            if(node->parent->children[0].get() == node) //is node NW child
                return nNode->children[2].get();

            if(node->parent->children[1].get() == node)
                return nNode->children[3].get();
        }

        //find southern neighbour
        case Direction::SOUTH:
        {
            if(node->parent->children[0].get() == node) // is node NW child
                return node->parent->children[2].get();

            if(node->parent->children[1].get() == node) // is node NE child
                return node->parent->children[3].get();

            tNode *nNode = findGreaterEqualNeighbour(node->parent, direction);
            if(nNode == nullptr || isLeaf(nNode))
                return nNode;

            if(node->parent->children[2].get() == node) // is node SW child
                return nNode->children[0].get();

            if(node->parent->children[3].get() == node)
                return nNode->children[1].get();
        }

        //find western neighbour
        case Direction::WEST:
        {
            if(node->parent->children[1].get() == node) // is node NE child
                return node->parent->children[0].get();

            if(node->parent->children[3].get() == node) // is node SE child
                return node->parent->children[2].get();

            tNode *nNode = findGreaterEqualNeighbour(node->parent, direction);
            if(nNode == nullptr || isLeaf(nNode))
                return nNode;

            if(node->parent->children[0].get() == node) // is node NW child
                return nNode->children[1].get();

            if(node->parent->children[2].get() == node)
                return nNode->children[3].get();
        }

        //find eastern neighbour
        case Direction::EAST:
        {
            if(node->parent->children[0].get() == node)
                return node->parent->children[1].get();

            if(node->parent->children[2].get() == node)
                return node->parent->children[3].get();

            tNode *nNode = findGreaterEqualNeighbour(node->parent, direction);

            if(nNode == nullptr || isLeaf(nNode))
                return nNode;

            if(node->parent->children[1].get() == node)
                return nNode->children[0].get();

            if(node->parent->children[3].get() == node)
                return nNode->children[2].get();
        }

        default:
        {
            throw("ERROR: Unknown direction option!");
            return nullptr;
        }
    }
}

std::vector<TMesh::tNode *> TMesh::getLeaves()
{
    if(m_leaves.empty())
        return findLeaves(m_Root.get(), m_Box);
    else
        return m_leaves;
}

TMeshBoundingBox TMesh::computeBox(const TMeshBoundingBox &box, int i) const
{
    //the initial coord of the bounding box that will be subdivied
    auto origin = box.getTopLeft();
    //new size of the bounding box for the child
    auto childSize = box.getSize() / static_cast<glm::f32>(2);
    //based on index i choosing which node will be subdivided next
    switch(i)
    {
        case 0: // North west node
            return TMeshBoundingBox(origin, childSize);
        case 1: // North east node
            return TMeshBoundingBox(glm::f32vec2(origin.x+childSize.x, origin.y), childSize);
        case 2: // South west node
            return TMeshBoundingBox(glm::f32vec2(origin.x, origin.y+childSize.y), childSize) ;
        case 3: // South east node
            return TMeshBoundingBox(glm::f32vec2(origin.x+childSize.x, origin.y+childSize.y), childSize);
        default:
            assert(false && "ASSERT: <computeBox(...)>: invalid child index!");
            return TMeshBoundingBox();
    }
}

int TMesh::getQuadrant(const TMeshBoundingBox &nodeBox, const TMeshBoundingBox &valBox) const
{
    auto center = nodeBox.getCenter();
    // checking in which part we are West or East
    // West
    if(valBox.getRight() < center.x)
    {
        //checking North West quadrant
        if(valBox.getBottom() < center.y)
            return 0;
        //checking South West quadrant
        else if(valBox.top >= center.y)
            return 2;
        //nothing was found in both quadrants
        else
            return -1;
    }
    //East
    else if(valBox.left >= center.x)
    {
        //checking North East quadrant
        if(valBox.getBottom() < center.y)
            return 1;
        //checking South East quadrant
        else if(valBox.top >= center.y)
            return 3;
        //nothing was found in both quadrants
        else
            return -1;
    }
    //nothing was found in all quadrants
    else
        return -1;
}

std::vector<TMesh::tNode *> TMesh::getNeighbours(TMesh::tNode *node, Direction direction)
{
    tNode *neighbour = findGreaterEqualNeighbour(node, direction);
    std::vector<tNode *> neighbours = findSmallerNeighbours(neighbour, direction);
    return neighbours;
}

} //namespace hfim2D
