/*  AI RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
    Basé sur https://github.com/hiveeyes/osbh-audioanalyzer/tree/df4caf7810c1afb3f4c2074d80d163fbace2183f - OSBH - Licence MIT
*/

#include "audio_functions.h"

String ai_state = "unkown";

#include <LITTLEFS.h>
using namespace std;
float windowLength = 2;
float samplingRate = I2S_SAMPLE_RATE;
const string states[] = {"test", "active", "dormant", "pre-swarm", "swarm", "sick_v", "sick_w", "sick_n", "theft", "collapsed", "missing_queen", "queen_hatching"};

/* AI class */
class Filter
{
    //Number of second order sections
    int size;

    //Filter coefficients, organized in second order sections:
    //b01 b11 b21 1 a11 a21
    //b02 b12 b22 1 a12 a22
    vector<vector<float> > coeffs;

    //Filter gain
    float g;

    //Buffers
    vector<vector<float> > z;

  public:
    Filter (vector<vector<float> > coeffs, float g);
    float filter (float);

};

class FeatureExtractor {

    //Filters
    std::vector<Filter> filters;

    //window length in samples
    int windowSamples;

    //updated sample count
    int sampleCount;

    //Frequency band energy
    std::vector<float> energy;

    //Energy value for normalization
    float totalEnergy;

    //ready flag
    bool ready;

  public:
    FeatureExtractor (std::vector<Filter> &filters,  float samplingRate, float windowLength);
    FeatureExtractor (std::vector<Filter> &filters,  int windowSamples);
    void update(float value);
    bool isReady();
    std::vector<float> getEnergy();
    float getTotalEnergy();
    void clearEnergy();

};

class Classifier {

    //Classifier type
    int classifierType;

    //Decision Tree
    vector<vector<float>> decisionTree;

    //Linear regression
    vector<vector<float>> linearComb;

  public:
    Classifier (string type, string s);
    void tree_parse (string s);
    void logistic_parse (string s);
    int classify (std::vector<float> energy);

};
/* End of AI class */

FeatureExtractor::FeatureExtractor(std::vector<Filter> &filters, float samplingRate, float windowLength)
{
  this->filters = filters;
  this->windowSamples = int (samplingRate * windowLength);
  this->sampleCount = 0;
  this->energy.resize(filters.size());
  std::fill(this->energy.begin(), this->energy.end(), 0);
  this->totalEnergy = 0;
  this->ready = false;
}

FeatureExtractor::FeatureExtractor(std::vector<Filter> &filters, int windowSamples)
{
  this->filters = filters;
  this->windowSamples = windowSamples;
  this->sampleCount = 0;
  this->energy.resize(filters.size());
  std::fill(this->energy.begin(), this->energy.end(), 0);
  this->totalEnergy = 0;
  this->ready = false;
}

void FeatureExtractor::update (float value)
{
  float energy_local;
  for (int i = 0; i < filters.size(); i++)
  {
    energy_local = filters[i].filter(value);
    energy[i] += pow(energy_local, 2);
  }

  totalEnergy += pow(value, 2);
  sampleCount = sampleCount + 1;

  //If sample count has reached window length, normalize, set flag as ready and reset sample count
  if (sampleCount >= windowSamples)
  {
    for (int i = 0; i < filters.size(); i++)
    {
      energy[i] = energy[i] / totalEnergy;
    }
    ready = true;
    sampleCount = 0;
  }

}

bool FeatureExtractor::isReady()
{
  return ready;
}

std::vector<float> FeatureExtractor::getEnergy()
{
  ready = false;
  return energy;
}

float FeatureExtractor::getTotalEnergy()
{
  return totalEnergy;
}

void FeatureExtractor::clearEnergy()
{
  std::fill(energy.begin(), energy.end(), 0);
  totalEnergy = 0;
}

Filter::Filter (vector<vector<float> > coeffs, float g) {
  this->g = g;
  this->coeffs = coeffs;
  this->size = coeffs.size();
  this->z.resize(size);

  //Set z to 0
  for (int i = 0; i < size; i++)
  {
    z[i].resize(2);
    std::fill(this->z[i].begin(), this->z[i].end(), 0);
  }
}

vector <Filter> createFilters()
{
  //Filter vector
  vector <Filter> filters;

  //Float vectors for storing coefficients
  vector<vector<float>> coeffs;

  //Filter
  Filter f(coeffs, 0);

  vector<float> sos(6);

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.988192e+00,  9.898294e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 5.581749e-03);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.980926e+00,  9.846599e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 7.670585e-03);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.975628e+00,  9.812284e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 9.517449e-03);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.963954e+00,  9.744001e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 1.287313e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.952814e+00,  9.685842e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 1.585273e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.937975e+00,  9.616016e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 1.929711e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.916886e+00,  9.527626e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 2.368217e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.888722e+00,  9.423521e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 2.891657e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.793798e+00,  9.140404e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 4.298210e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.758468e+00,  9.051555e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 4.747219e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.716855e+00,  8.954499e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 5.230716e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00, -1.539156e+00,  8.601357e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 6.993926e-02);
  filters.push_back(f);
  coeffs.clear();

  sos = {(float)1.000000e+00,  0.000000e+00, -1.000000e+00,  1.000000e+00,  1.077900e+00,  5.427943e-01};
  coeffs.push_back(sos);
  f = Filter(coeffs, 2.286031e-01);
  filters.push_back(f);
  coeffs.clear();

  return filters;
}

Classifier::Classifier (string type, string s) {
  if (type.compare("decision_tree") == 0)
  {
    this->tree_parse(s);
    this->classifierType = 0;
  }
  else if (type.compare("logistic") == 0)
  {
    this->logistic_parse(s);
    this->classifierType = 1;
  }
}

void Classifier::tree_parse (string s)
{
  //std::stringstream lineStream(s);
  string line;
  int linePosition;


  vector<float> node;

  //Divide by lines
  while ((linePosition = s.find('\n')) != -1)
    //while(getline(lineStream, line))
  {
    line = s.substr(0, linePosition + 1);
    //Check first letter. If feature, push a 0 and the feature number
    if (line.at(0) == 'f')
    {
      //Push 0 for feature
      node.push_back(0.0);
      //node.push_back(stof(line.substr(1,line.find(',')-1)));

      //Push feature number
      node.push_back((float)atof(line.substr(1, line.find(',') - 1).data()));

      //Push the rest from the first comma
      //std::stringstream tokenStream(line.substr(line.find(',')-1));
      string token;
      string values = line.substr(line.find(',') + 1);
      values = line.substr(line.find(',') + 1);
      int commaPosition;

      while ((commaPosition = values.find(',')) != -1 || (commaPosition = values.find('\n')) != -1)
        //while(getline(tokenStream, token, ','))
      {
        token = values.substr(0, commaPosition);
        node.push_back((float)atof(token.data()));
        values = values.substr(commaPosition + 1);
      }


    }
    //If state, push a 1 and the state number
    else if (line.at(0) == 's')
    {
      node.push_back(1.0);
      node.push_back((float)atof(line.substr(1).data()));
    }
    else {
      //  cout<<"Error with classifier input string\n";
      break;
    }
    decisionTree.push_back(node);
    s = s.substr(linePosition + 1);
    node.clear();
  }
}

void Classifier::logistic_parse (string s)
{
  //std::stringstream lineStream(s);
  string line;
  int linePosition;


  vector<float> node;

  //Divide by lines
  while ((linePosition = s.find('\n')) != -1)
    //while(getline(lineStream, line))
  {
    line = s.substr(0, linePosition + 1);
    //Check first letter. Must be an s
    if (line.at(0) == 's')
    {
      //Push the state number
      node.push_back((float)atof(line.substr(1).data()));

      //Push the rest from the first comma
      //std::stringstream tokenStream(line.substr(line.find(',')-1));
      string token;
      string values = line.substr(line.find(',') + 1);
      values = line.substr(line.find(',') + 1);
      int commaPosition;

      while ((commaPosition = values.find(',')) != -1 || (commaPosition = values.find('\n')) != -1)
        //while(getline(tokenStream, token, ','))
      {
        token = values.substr(0, commaPosition);
        node.push_back((float)atof(token.data()));
        values = values.substr(commaPosition + 1);
      }


    }
    else {
      //  cout<<"Error with classifier input string\n";
      break;
    }
    linearComb.push_back(node);
    s = s.substr(linePosition + 1);
    node.clear();
  }
}

int Classifier::classify(vector<float> energy)
{
  int node = 0;
  switch (classifierType) {

    //Decision tree
    case 0:
      while (true)
      {
        if (decisionTree[node][0] == 1)
        {
          return (int)decisionTree[node][1];
        }
        else if (energy[(int)decisionTree[node][1]] <= decisionTree[node][2]) {

          node = (int)decisionTree[node][3];

        }
        else {

          node = (int)decisionTree[node][4];
        }
      }
      break;

    //Regression
    case 1:

      vector<float> pred;
      vector<float> instanceFs;

      float maxF = -1000000;
      float max = -1000000;
      float current;
      float predSum;

      for (int i = 0; i < linearComb.size(); i++)
      {
        instanceFs.push_back(0);
      }

      for (int i = 1; i < linearComb[0].size(); i++)
      {
        predSum = 0;
        for (int j = 0; j < linearComb.size(); j++)
        {
          if (i > 1)
          {
            pred.push_back(linearComb[j][i]*energy[i - 2]);
          }
          else {
            pred.push_back(linearComb[j][1]);
          }
          predSum += pred[j];
        }
        predSum /= linearComb.size();

        for (int j = 0; j < linearComb.size(); j++)
        {
          instanceFs[j] += (pred[j] - predSum) * (linearComb.size() - 1) / linearComb.size();
        }
        pred.clear();

      }

      for (int i = 0; i < instanceFs.size(); i++)
      {
        if (instanceFs[i] > maxF)
        {
          maxF = instanceFs[i];
        }
      }

      for (int i = 0; i < instanceFs.size(); i++)
      {
        instanceFs[i] = exp(instanceFs[i] - maxF);
        //cout << instanceFs[i];
        //cout << "\n";
        if (instanceFs[i] > max) {
          max = instanceFs[i];
          node = linearComb[i][0];
        }
      }


      return node;

      break;
  }

}

float Filter::filter (float x)
{
  float y = g * x;

  float v = 0;

  //for each 2nd order section
  for (int f = 0; f < size; f++) {

    //a coefficients

    y = y - coeffs[f][4] * z[f][0];
    y = y - coeffs[f][5] * z[f][1];

    v = y;

    y = y * coeffs[f][0];

    //b coefficients
    y = y + coeffs[f][1] * z[f][0];
    y = y + coeffs[f][2] * z[f][1];

    //update buffer
    z[f][1] = z[f][0];
    z[f][0] = v;

  }

  return y;
}

int majorityVoting(std::vector<int> states)
{
  //Count all states
  std::array<int, 11> count = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for ( int i = 0; i < states.size(); i++ )
  {
    count[states[i]]++;
  }

  //Determine most repeated state
  int indexWinner = 1;
  for ( int i = 1; i < count.size(); i++ )
  {
    if ( count[i] > count[indexWinner] )
    {
      indexWinner = i;
    }
  }
  return indexWinner;
}

String ai() {
  
  recording_file_init();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(RECORD_TIME * 1000 + 2500);

  
  Serial.println("[AI] Starting");
  Classifier c = Classifier("logistic", "s1,3.03,4.21,-2,-0.54,0,-10.97,1.20,-5.76,0,4.15,-1.74,16.44,-35.67,-23.88\ns4,2.4,15,-33.46,-44.64,-2.5,1.91,-3.92,6.02,5.17,-14.3,35.72,-23.05,-22.39,6.87\ns9,-3.13,4.4,3.08,0.98,3.94,5.19,3.96,5.47,4.8,7.46,0.79,-44.22,24.39,41.2\ns10,3.88,-32.09,-0.18,1.81,-5.46,7.85,-8.62,-9.82,-1.14,-9.11,-5.44,2.34,9.25,-26.03\n");
  vector<Filter> filters = createFilters();
  FeatureExtractor fex(filters, samplingRate, windowLength);
  vector < vector < float > > energy;
  vector < int > DetectedStates;
  vector <float> energy_local;  
  File audio_file = LITTLEFS.open("/recording.wav", FILE_READ);
  if (audio_file != NULL)
  {
    // On dépasse le header du fichier wav
    audio_file.seek(45, SeekSet);
    char buffer_file[2];
    while (audio_file.readBytes(buffer_file, 2) == 2)
    {
      /*Serial.print(buffer_file[0], BIN);
      Serial.print(" ");
      Serial.print(buffer_file[1], BIN);
      Serial.print(" = ");*/
      int16_t y = (buffer_file[0] << 8) + buffer_file[1];
      //Serial.println(y, BIN);
      float x = (float)y/32768.0; 
      /*Serial.println(x, BIN);
      Serial.println("");
      Serial.println(y);
      Serial.println(x);
      Serial.println("");*/
      fex.update(x);

      if (fex.isReady()) {
        energy_local = fex.getEnergy();
        DetectedStates.push_back(c.classify(energy_local));
        fex.clearEnergy();
        energy.push_back(energy_local);

        if (DetectedStates.size() == RECORD_TIME / windowLength) {
          Serial.print("[AI] Last states average : ");
          Serial.println(states[majorityVoting(DetectedStates)].c_str());
          ai_state = states[majorityVoting(DetectedStates)].c_str();
          DetectedStates.clear();
        }
      }
    }
  } else {
    Serial.println("[AI] : Can't open audio file");
  }
  audio_file.close();
  return ai_state;
}
