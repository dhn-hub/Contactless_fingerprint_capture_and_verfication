#ifndef CLSEDGEMATCHER_H
#define CLSEDGEMATCHER_H

#include "myTypes.h"
#include "opencv2/opencv.hpp"
#include "CvPointOperations.h"
#include "cvGeometricCalculations.h"

using namespace cv;
using namespace std;
/************************************************************************/

#define RAD_TO_DEG (180.0/CV_PI)

/************************************************************************/

struct EdgeMatcherSegment;
struct EdgeMatcherEdge;
struct EdgeMatcherEdgePair;
struct EdgeMatcherIntersection;

/************************************************************************/

//! Enum zur Spezifikation von Segmenten, die eine Kante beruehren
typedef enum
{
  TOUCHES_NO_BORDER = 0,
  TOUCHES_ONE_BORDER = 1,
  LIES_ON_BORDER = 2,
  CUTS_FRAME_CORNER = 3
} EdgeMatcherBorderTouch;

//! Enum zur Spezifikation von Kanten-Typen
typedef enum
{
  EDGEMATCHER_EDGETYPE_EDGEPAIR = 0,
  EDGEMATCHER_EDGETYPE_IMGBORDER = 1,
  EDGEMATCHER_EDGETYPE_NONE = 2
} EdgeMatcherEdgeType;


//! Enum zur Spezifikation von Paarungsverfahren
typedef enum
{
  EDGEMATCHER_PAIRED_INVALID = 0,
  EDGEMATCHER_PAIRED_2EDGES = 1,
  EDGEMATCHER_PAIRED_1EDGE_WITHBORDER = 2,
  EDGEMATCHER_PAIRED_WITH_SHORT_BORDER_TOUCHING_EDGES = 3
} EdgeMatcherEdgePairingType;

/************************************************************************/


//! Struct describing dynamic thresholds, i.e. thresholds that depending on some other value
struct EdgeMatcherDynamicThreshold
{
  double parLeft;     //!< Left end of parameter window
  double valLeft;     //!< Value at left end of parameter window
  double parRight;    //!< Right end of parameter window
  double valRight;    //!< Value at right end of parameter window
  double scale;       //!< Pre-computed scale

  //-----------------------------------------------------------------------------
  //! @brief Standard constructor
  //-----------------------------------------------------------------------------
  EdgeMatcherDynamicThreshold();

  //-----------------------------------------------------------------------------
  //! @brief Parametrized constructor
  //!
  //! @param[in]  _parLeft    Left end of parameter window
  //! @param[in]  _valLeft    Value at left end of parameter window
  //! @param[in]  _parRight   Right end of parameter window
  //! @param[in]  _valRight   Value at right end of parameter window
  //-----------------------------------------------------------------------------
  EdgeMatcherDynamicThreshold(double _parLeft, double _valLeft, double _parRight, double _valRight);

  //-----------------------------------------------------------------------------
  //! @brief () Operator used for threshold computation
  //!
  //! @param[in]  _par    Parameter for which the threshold should be computed
  //-----------------------------------------------------------------------------
  double inline operator () (const double _par);
};

//=================================================================================================

//! Struct describing points on a blob contour; used for line segment generation
struct EdgeMatcherContourPoint
{
  int index;                  //!< Sequence index of contour point
  Vector2D coord;             //!< Coordinates of contour point
  Vector2D normal;            //!< Normal vector on contour point
  double angle;               //!< Tangent angle of contour point
  double angleAhead;          //!< Tangent angle ahead of the contour point
  double angleBehind;         //!< Tangent angle behind the contour point
  double angleDifference;     //!< Difference of 'ahead' and 'behind' tangent angles
  bool declaredAsLine;        //!< Flag indicating whether this contour point is part of a line segment

  //-----------------------------------------------------------------------------
  //! @brief Standard constructor
  //-----------------------------------------------------------------------------
  EdgeMatcherContourPoint();

  //-----------------------------------------------------------------------------
  //! @brief Method for setting basic data of a contour point
  //!
  //! @param[in]  _index  Sequence index of contour point
  //! @param[in]  _coord  Coordinates of contour point
  //-----------------------------------------------------------------------------
  inline void setBase(int _index, Point &_coord);

  //-----------------------------------------------------------------------------
  //! @brief Method for setting angle data of a contour point
  //!
  //! @param[in]  _vectorDiffAhead    Difference vector to contour points ahead of the point
  //! @param[in]  _vectorDiffBehind   Difference vector to contour points behind the point
  //-----------------------------------------------------------------------------
  inline void setAngle(Vector2D &_vectorDiffAhead, Vector2D &_vectorDiffBehind);

  //-----------------------------------------------------------------------------
  //! @brief Method for retrieving angle data of a contour point
  //!
  //! @param[in]  bForward    Flag indicating whether to retrieve the 'ahead' angle
  //-----------------------------------------------------------------------------
  inline double getAngle(bool bForward);

  //-----------------------------------------------------------------------------
  //! @brief Static method for comparing two contour points in terms of angle difference
  //!
  //! @param[in]  a   First contour point
  //! @param[in]  b   Second contour point
  //-----------------------------------------------------------------------------
  static bool compareUsingAngle(EdgeMatcherContourPoint *a, EdgeMatcherContourPoint *b);
};

//=================================================================================================

//! Struktur, welche die Parameter des Tool-Extractors enth&auml;lt.
struct EdgeMatcherParameters
{
  int     CONTOUR_MIN_TOTAL_POINTS;       //!< Minimum number of points within a contour
  int     CONTOUR_SMOOTH_POINTS;          //!< Number of pixels used for averaging angles, ...
  int     CONTOUR_BORDER_CROP;            //!< Minimaler Abstand von Geradensegmenten zum Bildrand, um zur Rekonstruktion von Kanten verwendet zu werden

  double  LINE_MAX_ANGLE_DEVIATION;       //!< Maximum angle deviation within a line segment
  int     LINE_MAX_ANGLE_DEVIATION_COUNT; //!< Maximum number of consecutive pixels with too high angle deviation within a line segment
  int     LINE_MIN_LENGTH;                //!< Minimal length of a line segment
  double  LINE_MAX_CURVINESS;             //!< Maximum curviness of a line segment
  double  LINE_MAX_REFLECTION_RATIO;      //!< The maximum ratio of a segment's contour points classified as reflection

  EdgeMatcherDynamicThreshold EDGE_MAX_SEGMENT_ANGLE_DYN;
  EdgeMatcherDynamicThreshold EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO_DYN;

  double  EDGE_MAX_SEGMENT_ANGLE;                     //!< Maximum angle between an edge and a line segment possibly used for refining the edge
  double  EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO;     //!< Maximum ratio of a segment's length and its endpoint distance
  double  EDGE_MAX_SEGMENT_RELATIVE_AXIS_DISTANCE;    //!< Maximum relative distance of line segment endpoints to edge axis for edge refinement
  int     EDGE_MIN_LENGTH;                            //!< Minimale L&auml;nge einer Kante

  double  EDGEPAIR_MAX_EDGE_PAIRING_ANGLE;                //!< Maximaler zwischen Kanten zulaessiger Winkel zur Paarung zu einem Tool
  double  EDGEPAIR_MIN_EDGE_DISTANCE;                     //!< Minimaler Abstand zweier paralleler Kanten, die gepaart werden koennen
  double  EDGEPAIR_MIN_BLOB_FILLING;                      //!< Minimum ratio white pixels required on a blob surface
  double  EDGEPAIR_MIN_LENGTH;                            //!< Minimum length of an edge pair (distance between contourTips)
};

//=================================================================================================

//! Klasse, welche die Kantenpaarungs-Algorithmen und die davon berechneten Ergebnisse aufnimmt
class CEdgeMatcher
{
public:
  //-----------------------------------------------------------------------------
  //! Konstruktor des EdgePair-Extractors. Anhand des uebergebenen Beispiel-Bilds wird
  //! die Bild-Groesse ermittelt und diverse Strukturen zur Speicherung von
  //! Zwischenergebnissen, zur Zeitmessung und zur Konfiguration des Tool-Extractors
  //! initialisiert.
  //!
  //! @param[in]  sampleImage     Beispiel-Bild anhand dessen z.B. die Bildgroesse ermittelt wird
  //-----------------------------------------------------------------------------
  CEdgeMatcher(Mat& sampleImage);

  //-----------------------------------------------------------------------------
  //! Destruktor des Tool-Extractors. Im Konstruktor erstellte Datenstrukturen
  //! werden wieder freigegeben.
  //!
  //! @param[in]     -
  //! @return        -
  //-----------------------------------------------------------------------------
  ~CEdgeMatcher();

  //-----------------------------------------------------------------------------
  //! Gibt einen Zeiger auf die allgemeine Struktur zurueck, welche die Parameter
  //! des ToolExtractors enthaelt. Die Parameter koennen direkt durch Setzen der
  //! Variablen in der verzeigerten Struktur ge&auml;ndert werden.
  //-----------------------------------------------------------------------------
  EdgeMatcherParameters* getParameters();

  //-----------------------------------------------------------------------------
  //! Beschraenkt den zu untersuchenden Bildbereich (ZUB) auf die &uuml;bergebene binaere
  //! Maske. Weisse Pixel geben dabei Bildbereiche an, die untersucht werden sollen.
  //! Die Maske wird verwendet, um Geraden-Segmente auszusortieren, die einen
  //! Mindestabstand zum ZUB unterschreiten, da hier (z.B. durch den Bildbeschnitt)
  //! die Orientierung eines Geraden-Segments verfaelscht werden kann. Um im Zuge
  //! der Tool-Extraktion den Abstand eines beliebigen Punkts zum ZUB zaegig auswerten
  //! zu k&ouml;nnen, wird die Maske durch mehrfache Schwarz-Propagation (Erode-Filter)
  //! verkleinert. Die Anzahl der Filter-Iterationen wird dabei dem Parameter
  //! "CONTOUR_BORDER_CROP" entnommen, welcher den Mindestabstand zum ZUB
  //! kodiert. Das Ergebnis ist somit eine binaere Maske, welche fuer einen Pixel direkt
  //! aussagt, ob dieser valide ist (weiss) oder dem ZUB zu nahe kommt (schwarz).
  //!
  //! @param[in]  mask     Maske anhand welcher der Bildbereich beschraenkt wird
  //-----------------------------------------------------------------------------
  void limitFrameTo(Mat& mask);

  //-----------------------------------------------------------------------------
  //! Extrahiert aus einem binaeren Blob-Bild die dort sichtbaren Tools. Fuer die
  //! Extraktion werden folgende Algorithmen nacheinander durchlaufen:
  //!
  //!     1. Lokalisieren von Konturen innerhalb des binaeren Blob-Bilds
  //!
  //!         1.1. Jede gefundene Kontur mit einer Mindestlaenge in eine Abfolge
  //!             von Geraden- und Kurven-Segmenten zerlegen
  //!
  //!     2. Anhand der Geraden-Segmenten etwaige Kanten rekonstruieren
  //!
  //!     3. Paarung von Kanten
  //!
  //!     4. Rekonstruktion etwaiger Tools anhand von gepaarten Kanten
  //!
  //!     5. Berechnung von Tool-Ueberkreuzungen
  //!
  //! @param[in]  mask        Binaeres Blob-Bild
  //-----------------------------------------------------------------------------
  void extractEdgePairsFromImage(Mat& mask, const Mat& reflectionsMask = Mat(), const Mat &bgrFrame = Mat());


  //-----------------------------------------------------------------------------
  //! Zeichnet die berechneten (Zwischen)Ergebnisse. Diese sind:
  //!
  //!     - Die gefundenen Tools (nachweisbare und Kontur-Tool-Spitzen, Tool-Achse)
  //!
  //!     - Die rekonstruierten und evtl. gepaarten Kanten
  //!
  //!     - Die Geraden- und Kurven-Segmente, in die die Konturen zerlegt wurden
  //!
  //! In das uebergebene Bild werden lediglich die Tools eingezeichnet. Die Kanten
  //! und Segmente werden in gesonderte Bild gezeichnet, die nur bei Bedarf angezeigt
  //! werden.
  //!
  //! @param[in,out]  frame   Das Ziel-Bild, in welches die gefundenen Tools eingezeichnet werden.
  //-----------------------------------------------------------------------------
  void renderResults();

  //-----------------------------------------------------------------------------
  //! Gibt im Laufe einer Frame-Verarbeitung belegten Speicher wieder frei. Bei
  //! diesem handelt es sich um die gefundenen Segmente, die rekonstruierten Kanten,
  //! die rekonstruierten Tools und die berechneten Tool-Ueberkreuzungen.
  //-----------------------------------------------------------------------------
  void endFrame();

  //-----------------------------------------------------------------------------
  //! Classifies a border point depending on its coordinates. Border classes are
  //! defined as follows:
  //!
  //! @param[in]  point   Der zu pruefende Punkt
  //! return              The border class of the point
  //-----------------------------------------------------------------------------
  static unsigned int getBorderType(const Vector2D &rPoint, const Size &rImageSize);

  std::vector<vector<Point>> contour;                                //!< Vector zur Aufnahme der aus einem Blob-Bild gewonnenen Konturen
  std::vector<EdgeMatcherSegment*> segment;                 //!< Vector zur Aufnahme der aus den Konturen extrahierten Segmente
  std::vector<EdgeMatcherEdge*> edge;                       //!< Vector zur Aufnahme der aus den Segmenten rekonstruierten Kanten
  std::vector<EdgeMatcherEdgePair*> edgePair;                       //!< Vector zur Aufnahme der aus den Kanten rekonstruierten Kantenpaaren
  std::vector<EdgeMatcherIntersection*> intersection;       //!< Vector zur Aufnahme der erkannten Tool-Ueberkreuzungen

  Mat contourBlobs;     //!< Kopie des Blob-Bilds f&uuml;r den OpenCV-Kontur-Scanner
  Mat edgeImage;        //!< Bild zur Visualisierung der gefundenen Kanten
  Mat contourImage;     //!< Bild zur Visualisierung der gefundenen Konturen
  Mat borderCrop;       //!< Graubild zur Kodierung des Bildbeschnitts
  Mat debugImage;       //!< Graubild zur Kodierung des Bildbeschnitts

  static int segmentCounter;
  static int edgeCounter;

protected:

  CvFont font;
  Size imageSize;               //!< x-y-Aufl&ouml;sung des Video-Frames
  vector<Vec4i> hierarchy;
  vector<vector<Point>> contourStorage;   //!< Struktur zur Verwaltung des OpenCV-Kontur-Scanners

  //-----------------------------------------------------------------------------
  //! Extracts line segments from a given contour, neglecting segments that
  //! hit reflections indicated by a given binary bitmap.
  //!
  //! @param[in]  contour             OpenCV point sequence of the contour
  //! @param[in]  parentContour       OpenCV point sequence of the parent contour (when processing holes)
  //! @param[in]  reflectionsMask     Binary bitmap containing all reflections
  //-----------------------------------------------------------------------------
  void segmentContour(vector<Point> contour, vector<Point> parentContour, const Mat& reflectionsMask);

  //-----------------------------------------------------------------------------
  //! Extracts a line segment from a contour starting from a specific seed point.
  //! The segment is generated by checking how far the start and end point can
  //! be displaced without breaking some angle deviation constraints. The resulting
  //! segment is inserted in the global segment list if it matches some additional
  //! criteria such as its curviness and the ratio of contained reflection pixels.
  //!
  //! @param[in]  pSeedPoint          Seedpoint of the segment
  //! @param[in]  rContourPoint       Vector of contour points
  //! @param[in]  pOcvContour         OpenCV point sequence of the contour
  //! @param[in]  pOcvParentContour   OpenCV point sequence of the parent contour (when processing holes)
  //! @param[in]  reflectionsMask     Binary bitmap indicating all reflections
  //-----------------------------------------------------------------------------
  void generateLineSegment(EdgeMatcherContourPoint *pSeedPoint, EdgeMatcherContourPoint *pContourPoint, vector<Point> pOcvContour, vector<Point> pOcvParentContour, const Mat& reflectionsMask);
  //-----------------------------------------------------------------------------
  //! Computes the displacement of a line segment end point in a given direction
  //! starting from a seed point on a contour. The displacement keeps on as long
  //! as a global angle deviation threshold is not surpassed for a number of pixels.
  //!
  //! @param[in]  pSeedPoint      Seedpoint of the segment
  //! @param[in]  rContourPoint   Vector of contour points
  //! @param[in]  bForward        Flag indicating in which direction to move ('forward' = 'true')
  //!
  //! @return     The (signed) number of contour points that the endpoint was displaced
  //-----------------------------------------------------------------------------
  int displaceSegmentEndPoint(EdgeMatcherContourPoint *pSeedPoint, EdgeMatcherContourPoint *pContourPoint, int iContourPoints, bool bForward);

  //-----------------------------------------------------------------------------
  //! Prueft, ob ein Punkt einen ausreichenden Abstand zum zu untersuchenden Bildbereich
  //! aufweist. Zur effizienten Pruefung, wird die zuvor mittels limitFrameTo() gesetzte
  //! und manipulierte binaere Maske verwendet.
  //!
  //! @param[in]  point   Der zu pruefende Punkt
  //! return              'true' falls der Abstand gross genug ist, 'false' sonst
  //-----------------------------------------------------------------------------
  bool isFarFromImageBorder(Vector2D point);

  //-----------------------------------------------------------------------------
  //! Rekonstruiert Kanten auf Basis der innerhalb einer Kontur zuvor extrahierten
  //! Segmente. Segmente, die keine Geraden-Segmente sind, ueber keine valide
  //! Orientierung verfuegen oder dem Rand des zu untersuchenden Bildbereichs zu
  //! nahe kommen, werden zunaechst aussoriert. Die verbleibenden Kanten werden nach
  //! absteigender Laenge sortiert und in einer doppelten Schleife durchlaufen:
  //! Dabei wird auf Basis des laengsten Geraden-Segments, das bislang keiner Kante
  //! zugewiesen ist, eine neue Kante erstellt. Diese wird, sofern m&ouml;glich, um die
  //! verbleibenden Geraden-Segmente erweitert, wobei im Falle einer erfolgreichen
  //! Erweiterung das jeweilige Geraden-Segment der Kante zugewiesen wird. Kanten
  //! werden nur beibehalten, sofern sie nach Abschluss der Rekonstruktion eine
  //! gewisse Mindestlaenge ueberschreiten.
  //!
  //-----------------------------------------------------------------------------
  void buildEdgesFromLineSegments();

  //-----------------------------------------------------------------------------
  //! Paart zuvor rekonstruierte Kanten. Hierfuer werden die Kanten zunaechst nach
  //! absteigender Laenge sortiert. Anschliessend wird fuer jede Kante A diejenige
  //! Partnerkante B ermittelt, welche gemaess einer Bewertungsfunktion die hoechste
  //! Qualitaet besitzt. Ist B bislang ungepaart oder verfuegt A aus Sicht von B
  //! ueber eine hoehere Qualitaet als Bs bisherige Partnerkante C, werden A und B
  //! gepaart. Eine Paarung von B und C wird dabei aufgeloest, wobei C vorerst
  //! ungepaart verbleibt.
  //!
  //! @param pairWithBorders  If true, pairs not-yet-paired non-border edges with border-edges.
  //!                         If false, pairs non-border edges with non-border edges.
  //!
  //-----------------------------------------------------------------------------
  void pairEdges(bool pairWithBorders,const Mat& blobImage = Mat(), const Mat& bgrFrame = Mat());

  //-----------------------------------------------------------------------------
  //! Berechnet die &Uuml;berkreuzungspunkte der zuvor gefundenen Tools. Hierf&uuml;r werden
  //! die Tools paarweise miteinander verglichen und der Schnittpunkt ihrer Achsen
  //! berechnet. Liegt dieser jeweils zwischen der Spitzen der Tools, handelt es
  //! sich um eine valide &Uuml;berkreuzung, welche in einem STL-Vector abgelegt wird.
  //!
  //-----------------------------------------------------------------------------
  void computeIntersections();
};

//! Struktur zur Repraesentation eines Geraden- oder Kurvensegments, in die eine
//! Objekt-Kontur im Laufe der Extraktion zerlegt wird. Je nach Segment-Typ
//! beinhalten nicht alle Variablen gueltige Werte.
struct EdgeMatcherSegment
{
  int id;
  vector<Point> parentContour;               //!< Kontur, aus welcher das Segment extrahiert wurde
  vector<Point> contour;                     //!< Kontur, aus welcher das Segment extrahiert wurde
  EdgeMatcherBorderTouch onBorder;  //!< Liegt das Segment auf einem Bildrand?
  //!< gar nicht, mit einem Punkt oder komplett?
  int startPoint;                     //!< Startindex des Segments in der OpenCV-Kontur-Punktliste
  int endPoint;                       //!< Endindex des Segments in der OpenCV-Kontur-Punktliste
  int points;                         //!< Anzahl des Konturpunkte im Segment
  Vector2D stableStartPoint;          //!< Stabiles Mittel der in der ersten Segmenthaelfte enthaltenen Konturpunkte
  Vector2D stableEndPoint;            //!< Stabiles Mittel der in der zweiten Segmenthaelfte enthaltenen Konturpunkte
  double  stablePointDistance;        //!< Abstand des stabilen Start- und Endpunkts
  double linearity;
  Vector2D normal;                    //!< Normalenvektor des Geraden-Segments (zeigt immer "aus", d.h. in Schwarzen)
  bool hasOrientation;                //!< Aussage, ob das Segment ueber eine gueltige Orientierung (Normalenvektor) verfuegt
  EdgeMatcherEdge* assignedEdge;    //!< Referenz auf die Kante, welche anhand dieses Segments rekonstruiert wurde

  //-----------------------------------------------------------------------------
  //! Default-Konstruktor eines Segments mit grundlegenden Initialisierungen.
  //!
  //-----------------------------------------------------------------------------
  EdgeMatcherSegment();

  //-----------------------------------------------------------------------------
  //! Parametrisierter Konstruktor eines Segments
  //!
  //! @param[in] _contour      Die Kontur, aus welcher das Segment extrahiert wird.
  //! @param[in] _type         Der Typ des Segments (Gerade oder Kurve)
  //! @param[in] _startPoint   Der Start-Index in der angegebenen Kontur, ab welchem das Segment beginnt
  //-----------------------------------------------------------------------------
  EdgeMatcherSegment(vector<Point> _contour, vector<Point> _parentContour, int _startPoint, int _points, Size &_imageSize);

  //-----------------------------------------------------------------------------
  //! Gibt den 'n'-ten Punkt eines Segments (ausgehend vom Startpunkt) zur&uuml;ck.
  //! Dabei wird 'n' auf das Intervall [0, points) reduziert, so dass z.B. n=-1
  //! den letzten Punkt des Segments zurueckgibt.
  //!
  //! @param[in] index        Der Index des zur&uuml;ckzugebenden Punkts ausgehend vom Startpunkt des Segments
  //! @return                 Der Punkt
  //-----------------------------------------------------------------------------
  Vector2D getPoint(int index);

  //-----------------------------------------------------------------------------
  //! Berechnet die Entfernung eines Punkts zum Geraden-Segment
  //!
  //! @param[in]  point   Der Punkt, dessen Abstand bestimmt werden soll
  //! @return             Der Abstand des Punkts
  //-----------------------------------------------------------------------------
  double getDistanceFromPoint(Vector2D* point);

  double getLength();

  //-----------------------------------------------------------------------------
  //! Berechnet den Normalenvektor eines Segments anhand zweier stabiler Punkte
  //! unter Beibehalt dessen allgemeiner Orientierung. Letzteres erfolgt, indem
  //! der neue Normalenvektor negiert wird, sofern sein Vektorprodukt zu einem
  //! evtl. zuvor gesetzten Normalenvektor negativ ist.
  //!
  //! @param[in]  a       Der stabile Startpunkt des Segments
  //! @param[in]  b       Der stabile Endpunkt des Segments
  //-----------------------------------------------------------------------------
  void updateNormal(Vector2D a, Vector2D b);
};

//! Struktur, die das STL-basierte Sortieren eines EdgeMatcherSegment-Vectors erm&ouml;glicht.
struct ToolExtractorSegmentSort
{
  bool operator()(EdgeMatcherSegment* a, EdgeMatcherSegment* b)
  {
    return (a->points > b->points);
  }
};

//! Struktur zur Repr&auml;sentation einer Kante, die im Laufe der Tool-Extraktion aus Geraden-Segmenten rekonstruiert wurde.
struct EdgeMatcherEdge
{
  int id;
  EdgeMatcherSegment* rootSegment;      //!< Das initiale Geradensegment, auf Basis dessen die Kante rekonstruiert wurde
  EdgeMatcherSegment matchingSegment;   //!< Ein fiktives ggf. durch Einbettung weiterer Segmente erweitertes Segment, welches die Kante repr&auml;sentiert
  EdgeMatcherEdge* pairingEdge;         //!< Die Kante, mit welcher diese Kante gepaart wurde
  double pairingEdgeQuality;              //!< Die Qualit&auml;t der mit dieser Kanten gepaarten Kante
  double length;                          //!< L&auml;nge der Kante
  EdgeMatcherEdgePair* assignedTool;        //!< Referenz auf das Tool, welches aus dieser Kante rekonstruiert wurde
  Vector2D startPoint;                    //!< Anfangspunkt der Kante
  Vector2D endPoint;                      //!< Endpunkt der Kante
  Scalar colour;                        //!< Farbe der Kante bei der Visualisierung
  bool edgeDrawn;                         //!< Indikator, ob die Kante bereits gezeichnet wurde
  EdgeMatcherEdgeType type;             //!< Toolkante oder Abschnitt vom Bildrand
  EdgeMatcherBorderTouch borderLevel;        //!< Liegt das Segment auf einem Bildrand?
  //!< gar nicht, mit einem Punkt oder komplett?
  double quality;                         //!< kurze Kanten, die den Bildrand ber�hren, sollen schlechter bewertet werden
  bool hasRefinement;

  //-----------------------------------------------------------------------------
  //! Default-Konstruktor einer Kante
  //!
  //-----------------------------------------------------------------------------
  EdgeMatcherEdge();

  //-----------------------------------------------------------------------------
  //! Parametrisierter Konstruktor einer Kante. Es werden grundlegende Variablen
  //! initialisiert und Werte von dem uebergebenen Vorbild-Segment uebernommen.
  //!
  //! @param[in]  _rootSegment    Das Geraden-Segment, das als Vorbild der Kante dient
  //-----------------------------------------------------------------------------
  //-----------------------------------------------------------------------------
  EdgeMatcherEdge(EdgeMatcherSegment* _rootSegment);

  //-----------------------------------------------------------------------------
  //! @brief Berechnet den nicht notwendigerweise vollst&auml;ndig korrekten Mittelpunkt einer Kante
  //!
  //! @return     Der berechnete Mittelpunkt
  //-----------------------------------------------------------------------------
  Vector2D getCenter();

  //-----------------------------------------------------------------------------
  //! @brief Abkuerzung fuer den Zugriff auf den Normalenvektor der Kante
  //!
  //! @return     Der Normalenvektor der Kante
  //-----------------------------------------------------------------------------
  Vector2D getNormal();

  //-----------------------------------------------------------------------------
  //! Berechnet die Entfernung eines Punkts zur Kante
  //!
  //! @param[in]  point   Der Punkt, dessen Entfernung bestimmt werden soll
  //! @return             Die berechnete Entfernung
  //-----------------------------------------------------------------------------
  double getDistanceFromPoint(Vector2D* point);

  //-----------------------------------------------------------------------------
  //! Erweitert, sofern moeglich, eine Kante um ein uebergebenes Geraden-Segment.
  //! Gegen eine Erweiterung sprechen:
  //!
  //!     - Ein zu grosser Winkelabstand beider Normalen-Vektoren
  //!     - Ein zu grosser euklidischer Abstand der Segment-Endpunkte zur Kanten-Achse
  //!     - Eine falsche Lage des Schnittpunkts der Kanten- und der Segment-Achse
  //!
  //! Findet eine Erweiterung statt, werden die Endpunkte sowie der Normalen-Vektor
  //! der Kante neu berechnet.
  //!
  //! @param[in,out]  segment     Das Geraden-Segment, welches in die Kante mit einbezogen werden soll
  //! @return                     'true' falls das Geraden-Segment in die Kante einbezogen wurde, 'false' sonst
  //-----------------------------------------------------------------------------
  bool refineUsingSegment(EdgeMatcherSegment* segment);

  void getEndpointMetrics(EdgeMatcherSegment* segment, double &rDistance, double &rAngle);

  //-----------------------------------------------------------------------------
  //! Berechnet die Qualitaet einer Kante zur Bildung eines Kanten-Paares. In die
  //! Qualitaet fliessen mit ein:
  //!
  //!     - Der Winkel-Abstand der Normalen-Vektoren
  //!     - Die Lage der Kanten-Endpunkte zur jeweils anderen Kanten-Achse
  //!     - Der Laengen-Unterschied der Kanten
  //!     - Der Abstand der Kanten-Mittelpunkte
  //!
  //! @param[in]  edge    Die zu pruefende Kante
  //! @return             Die Eignung der Kante
  //-----------------------------------------------------------------------------
  double testEdgeAsPair(EdgeMatcherEdge* edge, const Mat& blobImage = Mat(), const Mat& bgrFrame = Mat());

  //-----------------------------------------------------------------------------
  //! Generates pixel coordinate samples within a given convex polygon. The samples
  //! are cropped against a supplied image size and inserted into a referenced
  //! vector.
  //!
  //! @param[in]  rvPolyPoint     Vector of 2D coordinates describing the convec polygon
  //! @param[in]  rImageSize      Image constraints used for cropping the samples
  //! @param[in]  rvSamplePoint   Reference to the vector in which the samples are inserted
  //! @return     Die Eignung der Kante
  //-----------------------------------------------------------------------------
  void generateShapeSamples(std::vector< Vector2D > &rvPolyPoint, Size &rImageSize, std::vector< Vector2D > &rvSamplePoint);

  //-----------------------------------------------------------------------------
  //! Paart die Kante mit einer anderen
  //!
  //! @param[in,out]  edge      Die Kante, mit welcher die Kante gepaart werden soll
  //! @param[in]      quality   Die zuvor ueber testEdgeAsPair() ermittelte Qualitaet
  //-----------------------------------------------------------------------------
  void pairEdgeWith(EdgeMatcherEdge* edge, double quality);

  //-----------------------------------------------------------------------------
  //! @brief Hebt eine bestehende Kanten-Paarung auf
  //!
  //-----------------------------------------------------------------------------
  void unpairEdge();
};

//! Struktur, die das STL-basierte Sortieren eines EdgeMatcherEdge-Vectors erm&ouml;glicht.
struct ToolExtractorEdgeSort
{
  bool operator()(EdgeMatcherEdge* a, EdgeMatcherEdge* b)
  {
    return (a->length > b->length);
  }
};

//! Struktur zur Repr&auml;sentation eines Tools, welches durch Paarung zweier Kanten rekonstruiert wurde.
struct EdgeMatcherEdgePair
{    
  EdgeMatcherEdge* rootEdge;    //!< Eine der Kanten, auf Basis derer das Tool rekonstruiert wurde
  Vector2D axis;                  //!< Die berechnete Tool-Achse (vector aims from the origin towards the tip)
  Vector2D axisNormal;            //!< Der Normalenvektor der Tool-Achse
  Vector2D axisBase;              //!< Ein Punkt auf der Tool-Achse
  Vector2D virtualStartTipLongEdge; //!< Erster aus den gepaarten Kanten berechneter Tool-Endpunkt bezogen auf die l?ngere Kante (liegt auf der Tool-Achse)
  Vector2D virtualEndTipLongEdge;   //!< Zweiter aus den gepaarten Kanten berechneter Tool-Endpunkt bezogen auf die l?ngere Kante(liegt auf der Tool-Achse)
  Vector2D virtualStartTipShortEdge;    //!< Erster aus den gepaarten Kanten berechneter Tool-Endpunkt bezogen auf die k?rzere Kante (liegt auf der Tool-Achse)
  Vector2D virtualEndTipShortEdge;      //!< Zweiter aus den gepaarten Kanten berechneter Tool-Endpunkt bezogen auf die k?rzere Kante (liegt auf der Tool-Achse)
  Vector2D contourStartTip;       //!< Erweiterung des ersten Tool-Endpunkts unter Verwendung des Blob-Bilds
  Vector2D contourEndTip;         //!< Erweiterung des zweiten Tool-Endpunkts unter Verwendung des Blob-Bilds
  double virtualLengthLongEdge;   //!< Abstand der berechneten Tool-Endpunkte
  double contourLength;           //!< Abstand der erweiterten Tool-Endpunkte
  double openingAngle;            //!< The angle between edgePair's edge lines in rad (0 if edges parallel, pi/2 if perpendicular)
  EdgeMatcherEdgePairingType edgePairingType;           //!< Kantenpaarungstyp; 1: zwei Tool-Kanten, 2: eine Tool-Kante mit einer Bildkanten
  Mat blobs;                //!< Blob-Bild innerhalb dessen das Tool extrahiert wurde
  float customVal1, customVal0;   //!< User-defined values, used to add some additional temporary info.
  uint threshold;                 //!< Threshold at which this edge pair was found. If AWB is on, constant 1000 is added.

  //-----------------------------------------------------------------------------
  //! Konstruiert ein Tool auf Basis eines Kanten-Paares und eines binaeren Blob-Bilds.
  //! Dabei werden die Tool-Achse, der Hauptachsen-Stuetzpunkt, die nachweisbaren sowie
  //! die Kontur-Tool-Spitzen berechnet.
  //!
  //! @param[in,out]  _rootEdge   Ein Element des Kanten-Paares
  //! @param[in]      _mask       Das binaere Blob-Bild
  //-----------------------------------------------------------------------------
  EdgeMatcherEdgePair(EdgeMatcherEdge* _rootEdge, Mat& _mask);

  //-----------------------------------------------------------------------------
  //! Computes the axis of a edgePair using two edges.
  //!
  //! @param[in]  pEdgeA      First edge used for axis computation
  //! @param[in]  pEdgeB      Second edge used for axis computation
  //! @param[out] rAxisBase   2D base coordinate of axis
  //! @param[out] rAxis       2D axis direction
  //-----------------------------------------------------------------------------
  static inline void computeAxis(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rAxisBase, Vector2D &rAxis);

  //-----------------------------------------------------------------------------
  //! @brief Computes the tips of a edge pair using two edges and the respective axis.
  //!
  //! @param[in]  pEdgeA      First edge used for tip computation
  //! @param[in]  pEdgeB      Second edge used for tip computation
  //! @param[in]  rAxisBase   2D base coordinate of axis
  //! @param[in]  rAxis       2D axis direction
  //! @param[out] rStartTip   2D coordinate of first edge pair tip
  //! @param[out] rEndTip     2D coordinate of second edge pair tip
  //-----------------------------------------------------------------------------
  static inline void computeTips(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rAxisBase, Vector2D &rAxis, Vector2D &rStartTipLongEdge, Vector2D &rEndTipLongEdge, Vector2D &rStartTipShortEdge , Vector2D &rEndTipShortEdge);

  //-----------------------------------------------------------------------------
  //! @brief Computes the shape box of a edge pair using two edges and the tip points
  //!
  //! @param[in]  pEdgeA      First edge used for shape computation
  //! @param[in]  pEdgeB      Second edge used for shape computation
  //! @param[in]  rStartTip   2D coordinate of first edge pair tip
  //! @param[in]  rEndTip     2D coordinate of second edge pair tip
  //! @param[out] rvCoord     Referenced vector in which the shape point coordinates are stored
  //-----------------------------------------------------------------------------
  static inline void computeShape(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rStartTip, Vector2D &rEndTip, std::vector< Vector2D > &rvCoord);

  //-----------------------------------------------------------------------------
  //! Berechnet die HALBBreite eines Tools in einer bestimmten Entfernung zur Spitze.
  //! Dabei wird ausgehend von der uebergebenen Tool-Spitze und dem Abstand zu dieser
  //! die Distanz zu den Kanten berechnet, auf Basis derer das Tool urspruenglich
  //! rekonstruiert wurde.
  //!
  //! @param[in]  tip             Die Spitze, ausgehend von welcher die Tool-Breite berechnet werden soll
  //! @param[in]  tipDistance     Die Entfernung zur Spitze in Richtung Hauptachsenstuetzpunkt
  //! @return                     Die ermittelte Breite
  //-----------------------------------------------------------------------------
  double getWidth(Vector2D tip, double tipDistance = 0);

  //-----------------------------------------------------------------------------
  //! Prueft, ob ein uebergebener Punkt zwischen den Tool-Spitzen liegt.
  //!
  //! @param[in]  coordinates     Die zu pruefenden Koordinaten
  //! @return                     'true' falls der zu pruefende Punkt zwischen den Tool-Sptizen liegt, 'false' sonst
  //-----------------------------------------------------------------------------
  bool pointIsBetweenTips(Vector2D coordinates);

  //-----------------------------------------------------------------------------
  //! Draws the mask of this edgePair. The mask is symmetric (one circle at tip, one at origin),
  //! as we are not sure about the orientation here yet. Contour tip/origin is used here.
  //!
  //! @param[in]  maskImage           Target img
  //! @param[in]  widthCorrection     By how many px the edgePair should be made wider (~ dilation)
  //-----------------------------------------------------------------------------
  void renderToolMask(Mat& maskImage, double widthCorrection=0);

  //-----------------------------------------------------------------------------
  //! Zeichnet die (parallelen) Kanten von der Tool
  //!
  //! @param[in]  maskImage     Das Zielbild
  //! @param[in]  thickness     Linienbreite
  //! @param[in]  skipLine      0 = Beide Kanten Zeichen, 1/2 = Nur eine von denen
  //! @param[in]  angle         der Winkel der Normal der ausgewaehlten Kante (zeigt aus dem Tool heraus)
  //-----------------------------------------------------------------------------
  void renderToolEdges(Mat& maskImage, int thickness, int skipLine=0, float * normalAngle=0);

  //-----------------------------------------------------------------------------
  //! Returns the distance of a point to the edgePair axis line.
  //!
  //! @param[in]  pt   Point
  //! @return          Euclidean distance
  //-----------------------------------------------------------------------------
  double axisPtDist(const Vector2D & pt) const;

  //-----------------------------------------------------------------------------
  //! A heuristic to find out wheather two edge pairs are supposed to be the same:
  //! - their axis angle difference is less than 15 deg
  //! - there is an end-point of a edgePair which is less than 30px apart from the other edgePair's axis
  //!
  //! @param[in]     tmpEdgePair  First edgePair
  //! @param[in]     edgePair     Second edgePair
  //! @return        True if the edge pairs are probably the same.
  //-----------------------------------------------------------------------------
  static bool isToolDuplicit(EdgeMatcherEdgePair * tmpEdgePair, EdgeMatcherEdgePair * edgePair, int i=0, int j=0);

};

//! Struktur zur Repr&auml;sentation einer &Uuml;berkreuzung zweier Tools
struct EdgeMatcherIntersection
{
  EdgeMatcherEdgePair* a;   //!< Erstes an der &Uuml;berkreuzung beteiligtes Tool
  EdgeMatcherEdgePair* b;   //!< Zweites an der &Uuml;berkreuzung beteiligtes Tool
  Vector2D coordinates;   //!< Koordinaten der &Uuml;berkreuzung

  //-----------------------------------------------------------------------------
  //! @brief Parametrisierter Konstruktor einer Tool-Ueberkreuzung
  //!
  //! @param[in]  _a              Erstes Tool der Ueberkreuzung
  //! @param[in]  _b              Zweites Tool der Ueberkreuzung
  //! @param[in]  _coordinates    Die Kreuzungskoordinaten
  //-----------------------------------------------------------------------------
  EdgeMatcherIntersection(EdgeMatcherEdgePair* _a, EdgeMatcherEdgePair* _b, Vector2D _coordinates);
};

#endif // CLSEDGEMATCHER_H
