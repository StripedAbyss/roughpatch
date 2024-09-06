#pragma once
#include<vector>
#include<iostream>
#include <fstream>

// �������ڵ���
class TreeNode {
public:
    int id; // �ڵ��ʶ��
    std::vector<TreeNode*> children; // �ӽڵ��б�

    // ���캯������ʼ���ڵ��ʶ��
    TreeNode(int tid) : id(tid) {}
    void generateNodeGML(std::ofstream& outfile);
    void generateEdgeGML(std::ofstream& outfile);
    void Convert_GmlTree_To_GML();
};

// ��������
class Tree {
public:
    TreeNode* root; // ���ڵ�ָ��

    // ���캯������ʼ�����ڵ�Ϊ��
    Tree() : root(nullptr) {}

    void insert(int parentId, int Id);
    void insertRec(TreeNode* node, int parentId, int Id);
    void printTree(TreeNode* node);
};