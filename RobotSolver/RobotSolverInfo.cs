using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace RobotSolver
{
    public class RobotSolverInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "RobotSolver";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("c546f400-0e7e-4fe6-b08f-b8d718f44e01");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
