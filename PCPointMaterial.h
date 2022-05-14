//
// Created by haocheng yang on 2-5-22.
//

#ifndef MATERIALFROMPOINTCLOUD_MATERIAL_H
#define MATERIALFROMPOINTCLOUD_MATERIAL_H

#include <ostream>
#include "Vector3.hpp"

namespace pcmattex {




    class PCPointMaterial {


    private:

        float raw_intensity;
        Vector3 rgb;
        Vector3 point_coordinate;

        float corrected_intensity;

        float angle_of_incidence;

        Vector3 scanner_position;

        Vector3 point_normal;
        Vector3 laser_direction;


        float emissivity;
        float albedo;
        float reflectance;


    public:

        PCPointMaterial();

        bool materialIsNaN();

        float getRawIntensity() const;

        void setRawIntensity(float raw_intensity);

        const Vector3 &getRgb() const;

        void setRgb(const Vector3 &rgb);

        const Vector3 &getPointCoordinate() const;

        void setPointCoordinate(const Vector3 &point_coordinate);

        float getCorrectedIntensity() const;

        void setCorrectedIntensity(float corrected_intensity);

        float getAngleOfIncidence() const;

        void setAngleOfIncidence(float angle_of_incidence);

        const Vector3 &getScannerPosition() const;

        void setScannerPosition(const Vector3 &scanner_position);

        const Vector3 &getNormalDirection() const;

        void setNormalDirection(const Vector3 &normal_direction);

        const Vector3 &getLaserDirection() const;

        void setLaserDirection(const Vector3 &laser_direction);

        float getEmissivity() const;

        void setEmissivity(float emissivity);

        float getAlbedo() const;

        void setAlbedo(float albedo);

        float getReflectance() const;

        void setReflectance(float reflectance);

        friend std::ostream &operator<<(std::ostream &os, const PCPointMaterial &material);

    };

}


#endif //MATERIALFROMPOINTCLOUD_MATERIAL_H
