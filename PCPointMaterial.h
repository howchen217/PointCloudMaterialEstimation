//
// Created by haocheng yang on 2-5-22.
//

#ifndef MATERIALFROMPOINTCLOUD_MATERIAL_H
#define MATERIALFROMPOINTCLOUD_MATERIAL_H

#include <ostream>
#include "Vector3.hpp"

    /**
     * \brief Class containing all information that could be associated to a point cloud point.
     *
     * It contains input data such as raw intensity and rgb, intermediate data as results of calculation such as
     * corrected intensity and angle of incidence, as well as the final material data such as emissivity, albedo and
     * reflectance.
     */
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
        /**
         * \brief Default constructor.
         *
         * Default constructor of PCPointMaterial.
         */
        PCPointMaterial();

        /**
         * \brief Check if resulting material properties has NaN values.
         *
         * Check if emissivity, albedo or reflectance is NaN.
         *
         * @return True if either one of the material property is NaN.
         */
        bool materialIsNaN();

        /**
         * \brief Get the input raw intensity of the point cloud point.
         *
         * @return The input raw intensity.
         */
        float getRawIntensity() const;

        /**
         * \brief Set the raw intensity.
         *
         * @param raw_intensity Input raw intensity.
         */
        void setRawIntensity(float raw_intensity);

        /**
         * \brief Get the RGB value of the point cloud point.
         *
         * @return The RGB value of point cloud point.
         */
        const Vector3 &getRgb() const;

        /**
         * \brief Set the RGB value of the point cloud point.
         *
         * @param rgb The RGB value to set.
         */
        void setRgb(const Vector3 &rgb);

        /**
         * \brief Get the point cloud point coordinate.
         *
         * @return The point cloud point coordinate.
         */
        const Vector3 &getPointCoordinate() const;

        /**
         * \brief Set the point cloud point coordinate.
         *
         * @param point_coordinate The point cloud point coordinate to Set.
         */
        void setPointCoordinate(const Vector3 &point_coordinate);

        /**
         * \brief Get the Corrected Intensity of the point cloud point.
         *
         * @return The corrected Intensity of the point cloud point.
         */
        float getCorrectedIntensity() const;

        /**
         * \brief Set the Corrected Intensity of the point cloud point.
         *
         * @param corrected_intensity The corrected intensity to set.
         */
        void setCorrectedIntensity(float corrected_intensity);

        /**
         * \brief Get the angle of incidence of the point cloud point.
         *
         * @return The angle of incidence of the point cloud point.
         */
        float getAngleOfIncidence() const;

        /**
         * \brief Set the angle of incidence of the point cloud point.
         *
         * @param angle_of_incidence The angle of incidence to set.
         */
        void setAngleOfIncidence(float angle_of_incidence);

        /**
         * \brief Get the scanner position of the point cloud point.
         *
         * Most of the cases the scanner position is the origin (0, 0, 0).
         *
         * @return The scanner position of the point cloud point.
         */
        const Vector3 &getScannerPosition() const;

        /**
         * \brief Set the scanner position of the point cloud point.
         *
         * @param scanner_position The scanner position to set.
         */
        void setScannerPosition(const Vector3 &scanner_position);

        /**
         * \brief Get the normal direction of the point cloud point.
         *
         * @return The normal direction.
         */
        const Vector3 &getNormalDirection() const;

        /**
         * \brief Set the normal direction of the point cloud point.
         *
         * @param normal_direction The normal direction to set.
         */
        void setNormalDirection(const Vector3 &normal_direction);

        /**
         * \brief Get the laser direction of the point cloud point.
         *
         * @return The laser direction.
         */
        const Vector3 &getLaserDirection() const;

        /**
         * \brief Set the laser direction of the point cloud point.
         *
         * @param laser_direction The laser direction to set.
         */
        void setLaserDirection(const Vector3 &laser_direction);

        /**
         * \brief Get the emissivity of the point cloud point.
         *
         * Emissivity is one of the resulting material properties.
         * Defined as the amount of heat radiated by the material surface.
         *
         * @return The emissivity of the point cloud point.
         */
        float getEmissivity() const;

        /**
         * \brief Set the emissivity of the point cloud point.
         *
         * Emissivity is one of the resulting material properties.
         * Defined as the amount of heat radiated by the material surface.
         *
         * @param emissivity The emissivity to set.
         */
        void setEmissivity(float emissivity);

        /**
         * \brief Get the albedo of the point cloud point.
         *
         * Albedo is one of the resulting material properties.
         * Defined as the fraction of sunlight that a surface can reflect.
         *
         * @return The albedo of the point cloud point.
         */
        float getAlbedo() const;

        /**
         * \brief Set the albedo of the point cloud point.
         *
         * Albedo is one of the resulting material properties.
         * Defined as the fraction of sunlight that a surface can reflect.
         *
         * @param albedo The albedo to set.
         */
        void setAlbedo(float albedo);

        /**
         * \brief Get the reflectance of the point cloud point.
         *
         * Reflectance is one of the resulting material properties. Here it refers to diffuse reflectance,
         * the ratio of light energy reflected from a material relative to the amount of light incident on the material.
         *
         * @return The reflectance of the point cloud point.
         */
        float getReflectance() const;

        /**
         * \brief Set the reflectance of the point cloud point.
         *
         * Reflectance is one of the resulting material properties. Here it refers to diffuse reflectance,
         * the ratio of light energy reflected from a material relative to the amount of light incident on the material.
         *
         * @param reflectance The reflectance of the point cloud point to set.
         */
        void setReflectance(float reflectance);

        /**
         * \brief Log all the properties of the point cloud point.
         *
         * @param os Output stream.
         * @param material The PCPointMaterial.
         */
        friend std::ostream &operator<<(std::ostream &os, const PCPointMaterial &material);

    };


#endif //MATERIALFROMPOINTCLOUD_MATERIAL_H
