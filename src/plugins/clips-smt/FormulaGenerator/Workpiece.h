#ifndef WORKPIECE_H
#define WORKPIECE_H

#include <vector>
#include <map>
#include <string>



class Workpiece {
public:
    enum Color {NOTDEFINED = -1, NONE, RED, BLACK, SILVER, TRANSPARENT, BLUE, GREEN, YELLOW, ORANGE, GREY, 
        LAST_ENTRY = GREY};
    static int getMaxRingNumber();
    
    Workpiece();
    Workpiece(Color base, std::vector<Color> rings, Color cap);
    Workpiece(Color base, std::vector<Color> rings);
    Workpiece(Color base);

    virtual ~Workpiece();
    
    Color getBaseColor() const;
    Color getRingColor(int i) const;
    Color getCapColor() const;
    
    std::vector<Color> getRings() const;
    
    void setBaseColor(Color c);
    void setRingColor(Color c, int i);
    void setCapColor(Color c);
    
    void setRings(std::vector<Color> rings); 
    
    static Workpiece::Color intToColor(int color);
    
    static std::string toString(const Workpiece::Color color);
    std::string toString();

private:
    const static int maxRingNumber = 3;
    Color base = NONE;
    std::vector<Color> rings = std::vector<Color>(getMaxRingNumber(), Color(NONE));
    Color cap = NONE;

    //for toString
    static const std::map<Color, std::string> colorNames;
};

#endif /* WORKPIECE_H */

