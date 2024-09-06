#pragma once
#include <iostream>
#include <sstream>
#include <queue>
#include <opencv2/opencv.hpp>
#include "tree.h"
#include "libhgp.h"
using namespace libhgp;

struct Candidate {
    int CandidateId = 0;
    std::vector<Vector2d1> polygons;  // ��������еĶ���μ���
    double score;                    // ��������ĵ÷�
    std::vector<int> typenum;          //��������ж�������
    int previous = 1;
    // ���캯��
    Candidate(int id, const std::vector<Vector2d1>& polys, double sc, const std::vector<int>& tn, int p) : CandidateId(id), polygons(polys), score(sc), typenum(tn), previous(p) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //����
    }
    bool operator>(const Candidate& other) const {
        return score > other.score; //����
    }
};

class Beamsearch {
public:
    string image_path = "image_outputs";
    Tree gml_tree;
    vector<Vector2d1> origin_polygons;//δ�ּӹ�������μ���
    vector<Vector2d1> polygons;//����μ���
    vector<vector<Vector2d1>> process_solutions;
    vector<double> score;
    //void generateNodeGML(TreeNode* node, std::ofstream& outfile);
    //void generateEdgeGML(TreeNode* node, std::ofstream& outfile);
    //void Convert_GmlTree_To_GML();
    pair<double, int> calculateScore(const std::vector<Vector2d1>& polygons, int previous);
    bool doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2);
    Vector2d1 translatePolygon(const Vector2d1& polygon, double dx, double dy);
    std::vector<Vector2d1> beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect);
    void work();
    void test();
    void get_points_to_polygon();
    std::vector<Vector2d1> perior_geometry_put();
    void geometry_layer_output(vector<Vector2d1> a);
    void geometry_layer_save(vector<Vector2d1> a, int num, double score);
    void geometry_layer_save1(vector<Vector2d1> a, vector<Vector2d1> b);
    vector<double> GetScore();
    void PolygonModification();
    void PolygonModification1();
    void PolygonModification2();
};

