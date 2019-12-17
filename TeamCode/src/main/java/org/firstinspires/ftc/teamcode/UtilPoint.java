package org.firstinspires.ftc.teamcode;

@SuppressWarnings("unused")
final class UtilPoint
{
    // Polar coordinates (radius , angle)
    private double _r;
    private double _theta;

    // Cartesian coordinates (x , y)
    private double _x;
    private double _y;


    /**
     * Deals with the types of coordinates in R2 that this class will manage.
     */
    enum Type
    {
        POLAR ,
        CARTESIAN
    }


    /**
     * Takes coordinate values and a coordinate type and calculates the values for the other
     * coordinate type. It does so by calling a method.
     *
     * @param LEFT The left side of the coordinate
     * @param RIGHT The right side of the coordinate
     * @param TYPE The type of coordinate, Polar or Cartesian
     */
    UtilPoint(final double LEFT , final double RIGHT , final Type TYPE)
    {
        setCoordinates(LEFT , RIGHT , TYPE);
    }


    /**
     * Takes coordinate values and a coordinate type and calculates the values for the other
     * coordinate type.
     *
     * @param LEFT The left side of the coordinate
     * @param RIGHT The right side of the coordinate
     * @param TYPE The type of coordinate, Polar or Cartesian
     */
    private void setCoordinates(final double LEFT , final double RIGHT , final Type TYPE)
    {
        if(TYPE == Type.POLAR)
        {
            _r = LEFT;
            _theta = RIGHT;

            _x = _r * Math.cos(Math.toRadians(_theta));
            _y = _r * Math.sin(Math.toRadians(_theta));
        }
        else
        {
            _x = LEFT;
            //noinspection SuspiciousNameCombination
            _y = RIGHT;

            _r = Math.hypot(_x , _y);
            _theta = Math.toDegrees(Math.atan2(_y , _x));
        }

        // Constricts angle between 0 and 360
        _theta = trimAngle((int)_theta);
    }

    static int trimAngle(int angle)
    {
        angle %= 360;

        if(angle < 0)
            angle += 360;

        return angle;
    }
    /**
     * Get the polar coordinate radius.
     *
     * @return Returns the polar coordinate radius.
     */
    double r()
    {
        return _r;
    }


    /**
     * Get the polar coordinate angle.
     *
     * @return Returns the polar coordinate angle.
     */
    double theta()
    {
        return _theta;
    }


    /**
     * Sets the polar coordinate angle to a new value and recalculates the cartesian coordinates.
     *
     * @param NEW The value to change the polar coordinate angle to.
     */
    void setTheta(final double NEW)
    {
        _theta = NEW;
        setCoordinates(_r , _theta , Type.POLAR);
    }


    /**
     * Get the cartesian coordinate x value.
     *
     * @return Returns the cartesian coordinate x value.
     */
    double x()
    {
        return _x;
    }


    /**
     * Get the cartesian coordinate y value.
     *
     * @return Returns the cartesian coordinate y value.
     */
    double y()
    {
        return _y;
    }
}

