#include "beamsearch.h"
double boxx, boxy;
// �������ڶ��������ıȽϺ���
bool beamssort(const Vector2d1& poly1, const Vector2d1& poly2)
{
    // ��������洢����ε�����ֵ
    double xmax_score1, xmin_score1, xlength_score1, ylength_score1, area_score1;
    double xmax_score2, xmin_score2, xlength_score2, ylength_score2, area_score2;
    Vector2d max1, min1, max2, min2;
    // �����һ������εı߽������ֵ
    PL().HGP_2d_Polygon_Boundingbox_C(poly1, min1, max1);
    xmax_score1 = max1.x / boxx;
    xmin_score1 = min1.x / boxx;
    xlength_score1 = (max1.x - min1.x) / boxx;
    ylength_score1 = (max1.y - min1.y) / boxy;
    // ����ڶ�������εı߽������ֵ
    PL().HGP_2d_Polygon_Boundingbox_C(poly2, min2, max2);
    xmax_score2 = max2.x / boxx;
    xmin_score2 = min2.x / boxx;
    xlength_score2 = (max2.x - min2.x) / boxx;
    ylength_score2 = (max2.y - min2.y) / boxy;
    // �������ε��������ֵ
    area_score1 = PL().HGP_2D_Polygon_Area_C(poly1) / (boxx * (max1.y - min1.y));
    area_score2 = PL().HGP_2D_Polygon_Area_C(poly2) / (boxx * (max2.y - min2.y));
    // �������ε��ܵ÷�
    double score1 = xmax_score1 + xmin_score1 + xlength_score1 + ylength_score1 + area_score1;
    double score2 = xmax_score2 + xmin_score2 + xlength_score2 + ylength_score2 + area_score2;
    // �Ƚ���������ε��ܵ÷�
    return score1 > score2;
}

pair<double, int> Beamsearch::calculateScore(const std::vector<Vector2d1>& polygons, int previous)
{
    double areas = 0;
    double score = 0;
    int now = 0;
    Vector2d1 pys;
    for (auto it = polygons.begin(); it != polygons.end(); it++)
    {
        for (auto it1 = it->begin(); it1 != it->end(); ++it1) {
            pys.push_back((*it1));
        }
    }
    Vector2d max1, min1;
    PL().HGP_2d_Polygon_Boundingbox_C(pys, min1, max1);
    score = (max1.x - min1.x) * (max1.y - min1.y);
    return make_pair(boxx*boxy/score, 1);
    /*
    vector<Polygon_2> pys;
    vector<Polygon_2> ans;
    vector<Vector2d1> output;
    for (auto it = polygons.begin(); it != polygons.end(); it++)
    {
        pys.push_back(Convert_Vector2d1_to_Polygon_2(*it));
    }
    vector<Point_2> getit;
    CGAL_2D_Polygon_Dart_Sampling_b(pys, 0.5, getit, 100);//��ɢȡ�㣬�ж��Ƿ���ͼ����࣬���ص㼯
    ans = get_triangulation_net(getit, pys);//���ݵ㼯���ɾ���
    for (auto it = ans.begin(); it != ans.end(); it++) {
        output.push_back(Convert_Polygon_2_to_Vector2d1(*it));
    }
    //geometry_layer_output(output);
    for (auto it = ans.begin(); it != ans.end(); it++)
    {
        if (abs(it->area()) < 500) {//��С�ľ������迼��
            continue;
        }
        else {//������
            //���￼�ǵ��Ǿ�������������Χ��������ܳ����ܳ�
            areas += it->bbox().x_span() * it->bbox().y_span();
            double length = 0;
            now++;
            for (auto itt = it->edges_begin(); itt != it->edges_end(); itt++)
            {
                length += sqrt(itt->squared_length());
            }
            score += it->bbox().x_span() * it->bbox().y_span() * (it->bbox().x_span() + it->bbox().y_span()) * 2 / (length);
        }
    }
    */
    if (areas == 0)return make_pair(0, now);
    score /= areas;
    score = score * 0.9 + 0.1 * min(previous / now, 1);//��ʽ�������ɵ���
    return make_pair(score, now);//���ص÷��뾧������
}

bool Beamsearch::doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2) {//��ײ��⣬�������
    for (const Vector2d1& one_polygon : poly2) {
        if (PL().HGP_2D_Two_Polygons_Intersection_C(poly1, one_polygon) > 0) {
            return true; // ������ײ
        }
    }
    return false; // δ������ײ
}

Vector2d1 Beamsearch::translatePolygon(const Vector2d1& polygon, double dx, double dy) {
    std::vector<Vector2d> translatedVertices;
    // �������ж��㣬��ÿ���������ƽ�Ʋ���������ӵ��µĶ����б���
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {

        Vector2d translatedPoint((*it).x + dx, (*it).y + dy);
        translatedVertices.push_back(translatedPoint);
    }
    // ʹ���µĶ����б���һ���µ�Vector2d1���󲢷���
    return Vector2d1(translatedVertices.begin(), translatedVertices.end());
}

std::vector<Vector2d1> Beamsearch::beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect) {
    // ��ʼ����ѡ���������id
    int id = 0;
    // ����һ�����ȶ��У����ڴ洢��ѡ������������յ÷ִӸߵ�������
    std::priority_queue <Candidate, std::vector<Candidate>, less<Candidate>> candidates;
    // �������ڵ�
    Candidate root(id++, {}, 0.0, {}, 1);//�յĽڵ㣬û�м������Σ�����Ҳ��0
    // �����ڵ�����ѡ�����������
    candidates.push(root);
    // ����һ������ڵ㣬��־���ڵ�
    gml_tree.insert(-1, 0);//��϶������gml_tree���Ի󣬼���������ĵ�
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    // ����ԭʼ�����
    std::vector<Vector2d1> ori_Polygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), beamssort);// ������Ķ���ν������򣬰���һ���Ĺ������ﰴ��beamsort����õ��Ƕ��������

    // ����ÿ�������õĶ����
    for (int times = 0; times < sortedPolygons.size(); times++) {
        // ���ڴ洢��һ�ִεĺ�ѡ������������ȶ���
        std::priority_queue < Candidate, std::vector<Candidate>, less<Candidate>> nextCandidates;

        // ����ǰ�ִε�ÿ����ѡ�������
        while (!candidates.empty()) {
            // ��ȡ��ǰ���ŵĺ�ѡ�������
            Candidate candidate = candidates.top();
            candidates.pop();
            // ���Ʒ��ô����ı���
            int tab = 0;

            // �����ڵ�ǰλ�÷��ò�ͬ�Ķ����
            for (int i = 0; i < sortedPolygons.size(); i++) {
                // ���ö�����Ƿ��Ѿ������ڽ��������
                bool type_tab = 0;
                for (auto types : candidate.typenum) {
                    if (types == i) {
                        type_tab = 1;
                        break;
                    }
                }
                // ���������Ѿ������ڽ�������У�������
                if (type_tab != 0) {
                    continue;
                }
                // ���Ʒ��ô�������ೢ��3��
                if (tab < 3) tab++;
                else break;

                // ������η��������������
                Vector2d bomin, bomax, somin, somax;
                PL().HGP_2d_Polygon_Boundingbox_C(boundingRect, bomin, bomax);
                PL().HGP_2d_Polygon_Boundingbox_C(sortedPolygons[i], somin, somax);
                double dx = 0.0;
                double dy = bomax.y - somax.y;
                Vector2d1 finalPolygon = translatePolygon(sortedPolygons[i], dx, dy);
                // ������ú�����ײ�򳬳��߽磬������
                if (doPolygonsCollide2(finalPolygon, candidate.polygons) || somax.y > boxy) {
                    tab--;
                    continue;
                }
                // ʹ�ö��ַ�����ƽ�ƣ�ֱ��������ײ
                PL().HGP_2d_Polygon_Boundingbox_C(finalPolygon, bomin, bomax);
                double bottom_distance = bomin.y;
                bool judge = 1;
                double pymin = bomin.y;
                while (bottom_distance > 10) {
                    pymin -= bottom_distance;
                    if (pymin < 0) {
                        pymin += bottom_distance;
                        bottom_distance /= 2.0;
                        continue;
                    }
                    Vector2d1 translatedPolygon = translatePolygon(finalPolygon, 0.0, -bottom_distance);
                    judge = doPolygonsCollide2(translatedPolygon, candidate.polygons);
                    if (judge == true) {
                        pymin += bottom_distance;
                        bottom_distance /= 2.0;
                    }
                    else {
                        finalPolygon = translatedPolygon;
                    }
                }
                // �����µĺ�ѡ�������
                std::vector<Vector2d1> newPolygons = candidate.polygons;
                std::vector<int> temp = candidate.typenum;
                temp.push_back(i);//ѹ���¶�������
                newPolygons.push_back(finalPolygon);

                // �����µĽ�������ĵ÷�
                pair<double, int> sc_pv = calculateScore(newPolygons, candidate.previous);
                double newScore = sc_pv.first;

                // �������
                cout << "����id" << id << ":" << newScore << endl;
                // ����ͼ�񲢼�¼�÷�
                geometry_layer_save(newPolygons, id, newScore);
                score.push_back(newScore);
                process_solutions.push_back(newPolygons);
                // ���µĽ�����������ѡ����
                gml_tree.insert(candidate.CandidateId, id);//gml���Ĳ���
                Candidate son(id++, newPolygons, newScore, temp, sc_pv.second);
                nextCandidates.push(son);
                // ���ֺ�ѡ���еĴ�С�����������
                while (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }
        // ���º�ѡ�����������
        candidates = nextCandidates;
    }

    // ��ȡ��ѵĺ�ѡ�������
    while (candidates.size() > 1) {
        candidates.pop();
    }
    // ����������
    cout << "���score" << candidates.top().score << endl;
    vector<Vector2d1> a = origin_polygons;
    // ������ѽ��������ͼ��
    if (!candidates.top().polygons.empty()) {
        // ��ȡ��ѽ�������Ķ���κ�������ԭʼ�����е�����˳��
        vector<Vector2d1> final_plan = candidates.top().polygons;
        vector<int> final_nums = candidates.top().typenum;
        int i = 0;
        // ������ѽ�������е�ÿ�������
        for (auto it = final_plan.begin(); it != final_plan.end(); it++) {
            // ��ȡ��ǰ�������ԭʼ�����е�����
            int sort_index = final_nums[i];
            i++;
            // ���㵱ǰ������� x �� y ���ϵ�λ����
            double delta_x = ((*it).begin())->x - (sortedPolygons[sort_index].begin())->x;
            double delta_y = ((*it).begin())->y - (sortedPolygons[sort_index].begin())->y;
            cout << "deltas" << delta_x << " " << delta_y << endl;
            int j = 0;
            // ����ԭʼ����Ķ����
            for (auto ooo : ori_Polygons) {
                // �����ǰ������뵱ǰ������ԭʼ�������ͬһ�������
                if (ooo == sortedPolygons[sort_index]) {
                    // �Ըö���ε�ÿ���������λ�ƣ�ʹ���뵱ǰ����ε�λ�ö���
                    for (auto it = a[j].begin(); it != a[j].end(); it++) {
                        (*it).x += delta_x;
                        (*it).y += delta_y;
                    }
                }
                else {
                    j++;
                }
            }
        }
        // ������������ѽ��������ԭʼ�������ε�ͼ��
        geometry_layer_save1(final_plan, a);
    }
    // ������ѽ������
    return candidates.top().polygons;
}

void create_folder(string a) {//���ߺ����������ļ���
    string folderPath = "./" + a;
    CreateDirectory(folderPath.c_str(), NULL);
    return;
}

void Beamsearch::work() {
    string output_filename = image_path;//��Ŀǰ����£�������Ҫ��packing�仯���̵�ÿһ��ͼ����������image_path��һ���ļ��е�ַ��������Beamsearch����
    create_folder(output_filename);//�����������ͼ����ļ���
    SYSTEMTIME st;//��ȡʱ�䣬�ԶԱ����packing����ͼ����и���
    GetSystemTime(&st);
    string time_path = image_path + "/" + to_string(st.wYear) + "_" + to_string(st.wMonth) + "_" + to_string(st.wDay) + "_" + to_string(st.wHour) + "_" + to_string(st.wMinute) + "_" + to_string(st.wSecond);//�˴�packing���̵õ����ļ�����
    create_folder(time_path);//һ��packing��һ���ļ���
    this->image_path = "./" + time_path;
    get_points_to_polygon();//�����ļ����ڵ�Ԫ���ļ�
    //PolygonModification();//��Ԫ�����д��ϻ�
    Vector2d1 boundingRect;//Բ������2ά����ľ���
    int beamWidth = 3;//�����beamsearch�㷨������
    boundingRect.push_back(Vector2d(0, 0));
    boundingRect.push_back(Vector2d(boxx, 0));
    boundingRect.push_back(Vector2d(boxx, boxy));
    boundingRect.push_back(Vector2d(0, boxy));
    std::vector<Vector2d1> wtf = perior_geometry_put();//���ǵ�test.txt�е�Ԫ��������ͬ���Ǹ����ϣ����Ҫ������ͬ��Ԫ������ʹ�øú����ظ������ӦԪ��
    std::vector<Vector2d1> bestSolution = beamSearch(wtf, beamWidth, boundingRect);//�����㷨��beamsearch�㷨

    // �����ѽ������
    std::cout << "��ѽ��������" << std::endl;
    for (const Vector2d1& polygon : bestSolution) {
        // �������ε�����
        for (const Vector2d& point : polygon) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }
    if (bestSolution.empty())cout << "�޷����ɽ��������" << endl;//�������Ϊ�գ�������޳ɹ���������˵���⼸��ԭ������ô���ö��ᷢ����ײ��ͻ
    else geometry_layer_output(bestSolution);//�����������
}

void Beamsearch::test() {//���Ժ��������ڲ��Եľ���PolygonModification2()����������кܴ�����⣬������Ŀ�ͽ�չ������
    get_points_to_polygon();
    vector<Vector2d1> a = polygons;
    PolygonModification2();
    geometry_layer_save1(a, polygons);
}

void Beamsearch::get_points_to_polygon() {
    boxx = 700;
    boxy = 1000;
    string address = "test.txt";
    ifstream infile;
    infile.open(address);
    if (!infile.is_open()) {
        std::cout << "�ļ���ʧ��" << endl;
        return;
    }
    string line;
    while (getline(infile, line)) {//ÿ�δ��ļ���ȡһ��
        istringstream iss(line);
        Vector2d1 points;
        int n;
        iss >> n;
        double x, y;
        for (int i = 0; i < n; i++)
        {
            iss >> x >> y;
            points.push_back(Vector2d(x, y));
        }
        if (PL().HGP_2D_Polygon_Is_Clockwise_Oriented_C(points))//��ֹ���˳��ߵ����������ɵĶ�����Ǹ��ģ�����α���������Ϊ������ģ��������ǵÿ���˳��ʱ�����⣩
        {
            std::reverse(points.begin(), points.end());
        }
        polygons.push_back(points);
    }
    origin_polygons = polygons;//������ڴ����е��ʼ�Ķ������
}

std::vector<Vector2d1> Beamsearch::perior_geometry_put()//��������ظ���Ԫ��
{
    std::vector<Vector2d1> ans = polygons;
    std::cout << "ͼ����������Ƿ��ظ�" << endl;
    bool ques;
    cin >> ques;
    if (ques) {
        std::cout << "�ظ����м���" << endl;
        int n; cin >> n;
        while (n--) {
            int type;
            cin >> type;
            if (type > polygons.size()) {
                std::cout << "û�и����͵ļ��νṹŶ������������" << endl;
                n++;
                continue;
            }
            Vector2d1 temp = polygons[type - 1];
            ans.push_back(temp);
        }
    }
    return ans;
}

void Beamsearch::geometry_layer_output(vector<Vector2d1> a) {
    // ����ͼ��ĳߴ�

    // ����һ����ɫ��ͼ�񣬳ߴ�Ϊ(boxy, boxx / 2)����������ΪCV_64FC3����ʼֵΪ��ɫ
    cv::Mat rightimage(boxy, boxx / 2, CV_64FC3, cv::Scalar(0, 0, 0));

    // ���ƶ����
    for (const auto& polygon : a) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        const cv::Point* pts = points.data();
        int num_points = points.size();
        // ���ƶ��������
        cv::polylines(rightimage, &pts, &num_points, 1, true, cv::Scalar(255, 255, 255), 2);
    }

    // ���ҷ�תͼ��
    cv::Mat leftimage;
    cv::flip(rightimage, leftimage, 1);

    // ƴ������ͼ�񣬵õ��Գ�ͼ��
    cv::Mat symmetric_image;
    cv::hconcat(leftimage, rightimage, symmetric_image);

    // ����һ����ֱ��
    cv::Point point1(boxx / 2, boxy);
    cv::Point point2(boxx / 2, 0);
    cv::line(symmetric_image, point1, point2, cv::Scalar(0, 0, 255), 1);

    // ��ʾͼ��
    cv::imshow("Polygons", symmetric_image);
    cv::waitKey(0);
    return;
}

void Beamsearch::geometry_layer_save(vector<Vector2d1> a, int num, double score) {
    // ����ͼ��ĳߴ�
    string path = this->image_path;
    path = path + "/�ڵ�" + to_string(num) + "���֣�" + to_string(score) + ".jpg";
    cout << path << endl;

    // ����һ����ɫ��ͼ�񣬳ߴ�Ϊ(boxy, boxx / 2)����������ΪCV_64FC3����ʼֵΪ��ɫ
    cv::Mat rightimage(boxy, boxx / 2, CV_64FC3, cv::Scalar(0, 0, 0));

    // ���ƶ����
    for (const auto& polygon : a) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        const cv::Point* pts = points.data();
        int num_points = points.size();
        // ���ƶ��������
        cv::polylines(rightimage, &pts, &num_points, 1, true, cv::Scalar(255, 255, 255), 2);
    }

    // ���ҷ�תͼ��
    cv::Mat leftimage;
    cv::flip(rightimage, leftimage, 1);

    // ƴ������ͼ�񣬵õ��Գ�ͼ��
    cv::Mat symmetric_image;
    cv::hconcat(leftimage, rightimage, symmetric_image);

    // ����һ����ֱ��
    cv::Point point1(boxx / 2, boxy);
    cv::Point point2(boxx / 2, 0);
    cv::line(symmetric_image, point1, point2, cv::Scalar(0, 0, 255), 1);

    // ����ͼ��
    cv::imwrite(path, symmetric_image);
    cv::waitKey(0);
    return;
}

void Beamsearch::geometry_layer_save1(vector<Vector2d1> a, vector<Vector2d1> b) {
    // ����ͼ��ĳߴ�
    string path = this->image_path;
    string path1 = path + "/Roughing.jpg";
    string path2 = path + "/Finishing.jpg";
    string path3 = path + "/BothOfThem.jpg";

    // ����һ����ɫ��ͼ�񣬳ߴ�Ϊ(boxy, boxx / 2)����������ΪCV_8UC3����ʼֵΪ��ɫ
    cv::Mat rightimage1(boxy, boxx / 2, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<std::vector<cv::Point>> pts;

    // ���ƶ���Σ�ʹ�ú�ɫ���
    for (const auto& polygon : a) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        pts.push_back(points);
    }
    // �������
    cv::fillPoly(rightimage1, pts, cv::Scalar(0, 0, 0));

    // ���ҷ�תͼ��
    cv::Mat leftimage1;
    cv::flip(rightimage1, leftimage1, 1);

    // ƴ������ͼ�񣬵õ��Գ�ͼ��
    cv::Mat symmetric_image1;
    cv::hconcat(leftimage1, rightimage1, symmetric_image1);

    // ����һ����ֱ��
    cv::Point point1(boxx / 2, boxy);
    cv::Point point2(boxx / 2, 0);
    cv::line(symmetric_image1, point1, point2, cv::Scalar(0, 0, 255), 1);

    // ����ͼ��
    cv::imwrite(path1, symmetric_image1);

    // ����һ����ɫ��ͼ�񣬳ߴ�Ϊ(boxy, boxx / 2)����������ΪCV_8UC3����ʼֵΪ��ɫ
    cv::Mat rightimage2(boxy, boxx / 2, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<std::vector<cv::Point>> pts1;

    // ���ƶ���Σ�ʹ����ɫ���
    for (const auto& polygon : b) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        pts1.push_back(points);
    }
    // �������
    cv::fillPoly(rightimage2, pts1, cv::Scalar(255, 0, 0));

    // ���ҷ�תͼ��
    cv::Mat leftimage2;
    cv::flip(rightimage2, leftimage2, 1);

    // ƴ������ͼ�񣬵õ��Գ�ͼ��
    cv::Mat symmetric_image2;
    cv::hconcat(leftimage2, rightimage2, symmetric_image2);

    // ����һ����ֱ��
    cv::Point point3(boxx / 2, boxy);
    cv::Point point4(boxx / 2, 0);
    cv::line(symmetric_image2, point3, point4, cv::Scalar(0, 0, 255), 1);

    // ����ͼ��
    cv::imwrite(path2, symmetric_image2);

    // ����һ����ɫ��ͼ�񣬳ߴ�Ϊ(boxy, boxx / 2)����������ΪCV_8UC3����ʼֵΪ��ɫ
    cv::Mat rightimage3(boxy, boxx / 2, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<std::vector<cv::Point>> pts2;

    // ���ƶ���Σ�ʹ�ú�ɫ���
    for (const auto& polygon : a) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        pts2.push_back(points);
    }
    // ������Σ�ʹ�ú�ɫ���
    cv::fillPoly(rightimage3, pts2, cv::Scalar(0, 0, 0));

    std::vector<std::vector<cv::Point>> pts3;

    // ���ƶ���Σ�ʹ����ɫ���
    for (const auto& polygon : b) {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon) {
            // ����������ת��ΪOpenCVͼ������ϵ�е�����
            int x = vertex.x;
            int y = boxy - vertex.y; // ��OpenCV�У�ͼ���ԭ��λ�����Ͻǣ�������Ҫ��תy��
            cv::Point point(x, y);
            points.push_back(point);
        }
        pts3.push_back(points);
    }
    // ������Σ�ʹ����ɫ���
    cv::fillPoly(rightimage3, pts3, cv::Scalar(0, 0, 255));

    // ���ҷ�תͼ��
    cv::Mat leftimage3;
    cv::flip(rightimage3, leftimage3, 1);

    // ƴ������ͼ�񣬵õ��Գ�ͼ��
    cv::Mat symmetric_image3;
    cv::hconcat(leftimage3, rightimage3, symmetric_image3);

    // ����һ����ֱ��
    cv::Point point5(boxx / 2, boxy);
    cv::Point point6(boxx / 2, 0);
    cv::line(symmetric_image3, point5, point6, cv::Scalar(0, 0, 255), 1);

    // ����ͼ��
    cv::imwrite(path3, symmetric_image3);
    return;
}

vector<double> Beamsearch::GetScore()
{
    return score;
}

void Beamsearch::PolygonModification() {

}

void Beamsearch::PolygonModification1()
{
}

void Beamsearch::PolygonModification2()
{
}
