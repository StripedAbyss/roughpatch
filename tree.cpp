#include "tree.h"

// ����GML�ļ��нڵ�Ĳ���
void TreeNode::generateNodeGML(std::ofstream& outfile) {
    // д��ڵ�Ŀ�ʼ��Ǻ�ID������ļ���
    outfile << "   node" << std::endl;
    outfile << "   [" << std::endl;
    outfile << "      id " << id << std::endl;
    outfile << "   ]" << std::endl;
    // �ݹ鴦��ÿ���ӽڵ�
    for (TreeNode* child : children) {
        child->generateNodeGML(outfile);
    }
}

// ����GML�ļ��бߵĲ���
void TreeNode::generateEdgeGML(std::ofstream& outfile)
{
    // ������ǰ�ڵ��ÿ���ӽڵ�
    for (TreeNode* child : children) {
        // д��ߵĿ�ʼ����Լ�Դ�ڵ��Ŀ��ڵ㵽����ļ���
        outfile << "   edge" << std::endl;
        outfile << "   [" << std::endl;
        outfile << "      source " << id << std::endl;
        outfile << "      target " << child->id << std::endl;
        outfile << "   ]" << std::endl;
        // �ݹ鴦��ǰ�ӽڵ���ӽڵ�
        child->generateEdgeGML(outfile);
    }
}

// ��GmlTreeת��ΪGML��ʽ�ļ�
void TreeNode::Convert_GmlTree_To_GML() {
    // ���ļ���д��GML����
    std::ofstream outfile("tree.gml");
    if (outfile.is_open()) {
        // д��GML�ļ���ͷ����Ϣ������ͼ������
        outfile << "graph" << std::endl;
        outfile << "[" << std::endl;
        outfile << "   directed 0" << std::endl;

        // ����GML�ļ��нڵ�Ĳ���
        generateNodeGML(outfile);
        // ����GML�ļ��бߵĲ���
        generateEdgeGML(outfile);

        // д��GML�ļ���β����Ϣ���ر��ļ�
        outfile << "]" << std::endl;
        outfile.close();
    }
    else {
        // ����ļ���ʧ�ܣ������������Ϣ
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

// ����ڵ㷽�������ݸ��ڵ��ʶ�����½ڵ��ʶ�������½ڵ�
void Tree::insert(int parentId, int Id) {
    // ������ڵ�Ϊ�գ��Ҹ��ڵ��ʶ��Ϊ-1���򴴽����ڵ�
    if (root == nullptr) {
        if (parentId == -1) {
            root = new TreeNode(Id);
        }
        return;
    }

    // �ݹ�����½ڵ�
    insertRec(root, parentId, Id);
}

// �ݹ����ڵ㷽��
void Tree::insertRec(TreeNode* node, int parentId, int Id) {
    // �����ǰ�ڵ�ı�ʶ�����ڸ��ڵ��ʶ�������½ڵ����Ϊ��ǰ�ڵ���ӽڵ�
    if (node->id == parentId) {
        node->children.push_back(new TreeNode(Id));
    }
    else {
        // ����ݹ�����ӽڵ㲢�����½ڵ�
        for (TreeNode* child : node->children) {
            insertRec(child, parentId, Id);
        }
    }
}

// ��ӡ����������ָ���ڵ㿪ʼ�ݹ��ӡ���Ľṹ
void Tree::printTree(TreeNode* node) {
    if (node == nullptr) {
        return;
    }

    // ��ӡ��ǰ�ڵ�ı�ʶ���Լ��������ӽڵ�ı�ʶ��
    std::cout << node->id << ": ";
    for (TreeNode* child : node->children) {
        std::cout << child->id << " ";
    }
    std::cout << std::endl;

    // �ݹ��ӡÿ���ӽڵ�����ṹ
    for (TreeNode* child : node->children) {
        printTree(child);
    }
}

