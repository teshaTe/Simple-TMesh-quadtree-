#ifndef H_TMESH_RENDER_CLASS
#define H_TMESH_RENDER_CLASS

#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics.hpp>

#include "TMesh2D.h"

namespace tmesh2D {

template<class T, class S>
class TMeshRender
{
public:
    TMeshRender() = default;

    void createWindow(int resX, int resY, std::string windowName);
    void displayTree(T *tree, std::string fileName="", S *targLeaf=nullptr,
                     std::vector<S *> *leafNeighbours = nullptr)
                     { display(tree->getLeaves(), fileName, targLeaf, leafNeighbours); }
    void displayTree(std::vector<S *> leaves, std::string fileName="", S *targLeaf=nullptr,
                     std::vector<S *> *leafNeighbours = nullptr )
                     { display(leaves, fileName, targLeaf, leafNeighbours); }

    ~TMeshRender() = default;
private:

    void display(std::vector<S *> leaves, std::string fileName, S *targLeaf,
                 std::vector<S *> *leafNeighbours);
    void drawRectangle(S *node, sf::RenderTarget &canvas,
                       S *targLeaf = nullptr, std::vector<S *> *leafNeihbours = nullptr);

    sf::Image saveTreeImg(std::string fileName);

private:
    sf::RenderWindow *m_newWindow;
};

template<class T, class S>
void TMeshRender<T, S>::createWindow(int resX, int resY, std::string windowName)
{
    m_newWindow = new sf::RenderWindow(sf::VideoMode(resX, resY), windowName, sf::Style::Default);
    m_newWindow->setFramerateLimit(60);
}

template<class T, class S>
void TMeshRender<T, S>::display(std::vector<S *> leaves, std::string fileName,
                               S *targLeaf, std::vector<S *> *leafNeighbours)
{
    while(m_newWindow->isOpen())
    {
        sf::Event event;
        while(m_newWindow->pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
            {
                if(!fileName.empty())
                    saveTreeImg(fileName);
                m_newWindow->close();
            }
            else if(event.type == sf::Event::KeyPressed)
            {
                if(event.key.code == sf::Keyboard::Escape)
                {
                    if(!fileName.empty())
                        saveTreeImg(fileName);
                    m_newWindow->close();
                }
            }
        }
        m_newWindow->clear(sf::Color::White);

        for(size_t i = 0; i < leaves.size(); i++)
            drawRectangle(leaves[i], *m_newWindow, targLeaf, leafNeighbours);

        m_newWindow->display();
    }
}

template<class T, class S>
void TMeshRender<T,S>::drawRectangle(S *node, sf::RenderTarget &canvas, S *targLeaf, std::vector<S *> *leafNeihbours)
{
    float x = node->nodeBox.getTopLeft().x, y = node->nodeBox.getTopLeft().y;
    float width = node->nodeBox.getSize().x, height = node->nodeBox.getSize().y;

    sf::RectangleShape shape;
    shape.setPosition(x, y);
    shape.setSize( sf::Vector2f(width, height));
    shape.setOutlineThickness(1.0f);

    if(node->values.empty())
        shape.setOutlineColor(sf::Color(255, 0, 0));
    else
        shape.setOutlineColor(sf::Color(0, 0, 0));

    if(targLeaf != nullptr && (x == targLeaf->nodeBox.getTopLeft().x &&
                               y == targLeaf->nodeBox.getTopLeft().y))
        shape.setFillColor( sf::Color(0, 255, 0, 255));

    else if(leafNeihbours != nullptr)
    {
        for(size_t i = 0; i < leafNeihbours->size(); i++)
        {
            if(x == leafNeihbours->at(i)->nodeBox.getTopLeft().x &&
               y == leafNeihbours->at(i)->nodeBox.getTopLeft().y)
            {
                shape.setFillColor( sf::Color(255, 0, 0, 255));
                break;
            }
        }
    }
    else
        shape.setFillColor( sf::Color(255, 255, 255, 255));

    canvas.draw(shape);
}

template<class T, class S>
sf::Image TMeshRender<T, S>::saveTreeImg(std::string fileName)
{
    sf::Texture texture;
    texture.create(m_newWindow->getSize().x, m_newWindow->getSize().y);
    texture.update(*m_newWindow);
    sf::Image screenshot = texture.copyToImage();
    screenshot.saveToFile(fileName);

    return screenshot;
}

} //namespace tmesh2D

#endif //H_TMESH_RENDER_CLASS
